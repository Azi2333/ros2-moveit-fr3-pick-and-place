# ros2 pkg create --build-type ament_python lab3_pkg --dependencies rclpy pymoveit2
from threading import Thread

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node

from pymoveit2 import MoveIt2
from pymoveit2 import GripperInterface 
from pymoveit2.robots import franka as robot

import time

from geometry_msgs.msg import PoseStamped
from rclpy.qos import QoSProfile
import numpy as np

# Helper function to open the gripper
def open_gripper(node, gripper_interface):
    node.get_logger().info(f'Performing gripper action open')
    gripper_interface.open()
    gripper_interface.wait_until_executed()
    # Sometimes the gripper does not open fully, so we wait a bit
    time.sleep(2.0)


# Helper function to close the gripper
def close_gripper(node, gripper_interface):
    node.get_logger().info(f'Performing gripper action close')
    gripper_interface.close()
    gripper_interface.wait_until_executed()
    # Sometimes the gripper does not close fully, so we wait a bit
    time.sleep(2.0)


# Helper function to initialise the gripper
def init_gripper(node, gripper_interface):
    node.get_logger().info(f'Initialise gripper')
    open_gripper(node, gripper_interface)
    close_gripper(node, gripper_interface)


# Move the robot to a pose
def move_to_pose(node, moveit2, position, quat_xyzw, cartesian, cartesian_max_step, cartesian_fraction_threshold):
    node.get_logger().info(
        f"Moving to {{position: {list(position)}, quat_xyzw: {list(quat_xyzw)}}}"
    )

    moveit2.move_to_pose(
            position=position,
            quat_xyzw=quat_xyzw,
            cartesian=cartesian,
            cartesian_max_step=cartesian_max_step,
            cartesian_fraction_threshold=cartesian_fraction_threshold,
        )
    
    # Wait until the robot has moved
    moveit2.wait_until_executed()
    node.get_logger().info("Moved to position!")


def pnp(node, moveit2, gripper_interface, positions, quat_xyzw):

    # Get parameters
    cartesian = node.get_parameter("cartesian").get_parameter_value().bool_value
    cartesian_max_step = (
        node.get_parameter("cartesian_max_step").get_parameter_value().double_value
    )
    cartesian_fraction_threshold = (
        node.get_parameter("cartesian_fraction_threshold")
        .get_parameter_value()
        .double_value
    )

    # Move the robot to a before pick pose    
    move_to_pose(node, moveit2, positions[0], quat_xyzw, cartesian, cartesian_max_step, cartesian_fraction_threshold)
    # Open the gripper to pick up the object
    open_gripper(node, gripper_interface)

    # Pick up the object
    move_to_pose(node, moveit2, positions[1], quat_xyzw, cartesian, cartesian_max_step, cartesian_fraction_threshold)
    close_gripper(node, gripper_interface)

    # Move the robot to a before place pose
    move_to_pose(node, moveit2, positions[2], quat_xyzw, cartesian, cartesian_max_step, cartesian_fraction_threshold)

    # Place the object
    move_to_pose(node, moveit2, positions[3], quat_xyzw, cartesian, cartesian_max_step, cartesian_fraction_threshold)
    open_gripper(node, gripper_interface)

    # Move the robot to a before place pose
    move_to_pose(node, moveit2, positions[4], quat_xyzw, cartesian, cartesian_max_step, cartesian_fraction_threshold)

    # Move the robot to a neutral pose
    move_to_pose(node, moveit2, positions[5], quat_xyzw, cartesian, cartesian_max_step, cartesian_fraction_threshold)

class PnPNode(Node):
    def __init__(self):
        super().__init__('pnp_rf')
        # Create callback group that allows execution of callbacks in parallel without restrictions (this is rquired to allow pymoveit2 to work correctly)
        # If you are interested to find out what this is, check the following https://docs.ros.org/en/humble/How-To-Guides/Using-callback-groups.html
        self.callback_moveit_group = ReentrantCallbackGroup()
        self.callback_gripper_group = ReentrantCallbackGroup()
        self.callback_boxpose_group = ReentrantCallbackGroup()
        self.box1_pose = []
        self.box1_orien = []
        self.box2_pose = []
        self.box3_pose = []
        self.box4_pose = []
        self.box5_pose = []
        self.box6_pose = []
        self.box_pose_subscriber = self.create_subscription(
            msg_type=PoseStamped,
            topic='/model/box1/pose',
            callback=self.pose_callback,
            qos_profile=QoSProfile(depth=1),
            callback_group=self.callback_boxpose_group,
        )
        self.box_pose_subscriber = self.create_subscription(
            msg_type=PoseStamped,
            topic='/model/box2/pose',
            callback=self.pose_callback2,
            qos_profile=QoSProfile(depth=1),
            callback_group=self.callback_boxpose_group,
        )
        self.box_pose_subscriber = self.create_subscription(
            msg_type=PoseStamped,
            topic='model/box3/pose',
            callback=self.pose_callback3,
            qos_profile=QoSProfile(depth=1),
            callback_group=self.callback_boxpose_group,
        )
        self.box_pose_subscriber = self.create_subscription(
            msg_type=PoseStamped,
            topic='model/box4/pose',
            callback=self.pose_callback4,
            qos_profile=QoSProfile(depth=1),
            callback_group=self.callback_boxpose_group,
        )
        self.box_pose_subscriber = self.create_subscription(
            msg_type=PoseStamped,
            topic='model/box5/pose',
            callback=self.pose_callback5,
            qos_profile=QoSProfile(depth=1),
            callback_group=self.callback_boxpose_group,
        )
        self.box_pose_subscriber = self.create_subscription(
            msg_type=PoseStamped,
            topic='model/box6/pose',
            callback=self.pose_callback6,
            qos_profile=QoSProfile(depth=1),
            callback_group=self.callback_boxpose_group,
        )
        self.box_position = None  # To store the latest received position
        self.box_orientation = None  # To store the latest received orientation
        
        # Planner ID (one of the motion planning algorithms in the Context Tab in RViz)
        self.declare_parameter("planner_id", "RRTConnectkConfigDefault")
        
        # Declare parameters for cartesian planning (if True, the robot will move in a straight line to the goal; useful later in the lab excercise)
        self.declare_parameter("cartesian", False)
        self.declare_parameter("cartesian_max_step", 0.01) 
        self.declare_parameter("cartesian_fraction_threshold", 0.2)  

        self.declare_parameter("cartesian_jump_threshold", 0.0)
        self.declare_parameter("cartesian_avoid_collisions", False)    

        # Create MoveIt 2 interface
        self.moveit2 = MoveIt2(
            node=self,
            joint_names=robot.joint_names(),
            base_link_name=robot.base_link_name(),
            end_effector_name=robot.end_effector_name(),
            group_name=robot.MOVE_GROUP_ARM,
            callback_group=self.callback_moveit_group,
        )
        
        self.moveit2.planner_id = (
            self.get_parameter("planner_id").get_parameter_value().string_value
        )

        self.gripper_interface = GripperInterface(
            node=self,
            gripper_joint_names=robot.gripper_joint_names(),
            open_gripper_joint_positions=robot.OPEN_GRIPPER_JOINT_POSITIONS,
            closed_gripper_joint_positions=robot.CLOSED_GRIPPER_JOINT_POSITIONS,
            gripper_group_name=robot.MOVE_GROUP_GRIPPER,
            callback_group=self.callback_gripper_group,
            gripper_command_action_name="gripper_action_controller/gripper_cmd",
        )

    def pose_callback(self, msg):
        if msg.pose and msg.pose.position and msg.pose.position.x is not None:
            self.box1_pose = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
        else:
            self.get_logger().warn

    def pose_callback2(self, msg):
        if msg.pose and msg.pose.position and msg.pose.position.x is not None:
            self.box2_pose = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
        else:
            self.get_logger().warn

    def pose_callback3(self, msg):
        if msg.pose and msg.pose.position and msg.pose.position.x is not None:
            self.box3_pose = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
        else:
            self.get_logger().warn

    def pose_callback4(self, msg):
        if msg.pose and msg.pose.position and msg.pose.position.x is not None:
            self.box4_pose = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
        else:
            self.get_logger().warn

    def pose_callback5(self, msg):
        if msg.pose and msg.pose.position and msg.pose.position.x is not None:
            self.box5_pose = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
        else:
            self.get_logger().warn

    def pose_callback6(self, msg):
        if msg.pose and msg.pose.position and msg.pose.position.x is not None:
            self.box6_pose = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
        else:
            self.get_logger().warn




def main():
    rclpy.init()

    # Create node for this example
    node = PnPNode()

    # Spin the node in background thread(s) and wait a bit for initialization
    executor = rclpy.executors.MultiThreadedExecutor(2)
    executor.add_node(node)
    executor_thread = Thread(target=executor.spin, daemon=True, args=())
    executor_thread.start()
    node.create_rate(1.0).sleep()

    # Scale down velocity and acceleration of joints (percentage of maximum)
    node.moveit2.max_velocity = 0.15
    node.moveit2.max_acceleration = 0.15

    # Get parameters
    cartesian_jump_threshold = (
        node.get_parameter("cartesian_jump_threshold")
        .get_parameter_value()
        .double_value
    )
    cartesian_avoid_collisions = (
        node.get_parameter("cartesian_avoid_collisions")
        .get_parameter_value()
        .bool_value
    )

    # Set parameters for cartesian planning
    node.moveit2.cartesian_avoid_collisions = cartesian_avoid_collisions
    node.moveit2.cartesian_jump_threshold = cartesian_jump_threshold

    #####################################################################
    #####################################################################

    # Need to open and close gripper to initialise the controller (this is a workaround for a bug in the controller)
    init_gripper(node, node.gripper_interface)
    
    # #####################################################################
    # Create an array for positions
    positions_left = []
    # Rotation for pointing the end effector downwards
    quat_xyzw = [0.707, 0.707, 0.0, 0.0]  

    # Create an array for positions
    positions_left = []
    # Rotation for pointing the end effector downwards
    quat_xyzw = [1.0, 0.0, 0.0, 0.0]

    
    #####################################################################
    # Now pick up the box from the right side
    positions_box2 = [
        [0.6, 0.15, 0.2], # Move the robot to a before place pose
        [0.6, 0.15, 0.025], # Pick up the object
        [0.6, 0.15, 0.2], # Move the robot to a before place pose
        [-0.5, 0.25, 0.125], # Place the object
        [-0.5, 0.25, 0.2], # Move the robot to a before place pose
        [0.3, 0.0, 0.3] # Move the robot to a neutral pose
    ]
    
    positions_box2[0][:2]=node.box2_pose[:2]
    positions_box2[1][:2]=node.box2_pose[:2]
    positions_box2[2][:2]=node.box2_pose[:2]
    positions_box2[3][:2]=node.box1_pose[:2]
    positions_box2[4][:2]=node.box1_pose[:2]

    positions_box3 = [
        [0.45, 0.25, 0.2], # Move the robot to a before place pose
        [0.45, 0.25, 0.025], # Pick up the object
        [0.45, 0.25, 0.2], # Move the robot to a before place pose
        [-0.5, 0.25, 0.15], # Place the object   
        [-0.5, 0.25, 0.3], # Move the robot to a before place pose
        [0.3, 0.0, 0.3] # Move the robot to a neutral pose
    ]
    positions_box3[0][:2]=node.box3_pose[:2]
    positions_box3[1][:2]=node.box3_pose[:2]
    positions_box3[2][:2]=node.box3_pose[:2]
    positions_box3[3][:2]=node.box1_pose[:2]
    positions_box3[4][:2]=node.box1_pose[:2]

    positions_box4 = [
        [-0.55, -0.45, 0.2], # Move the robot to a before place pose
        [-0.55, -0.45, 0.025], # Pick up the object
        [-0.55, -0.45, 0.3], # Move the robot to a before place pose
        [-0.5, 0.25, 0.175], # Place the object
        [-0.5, 0.25, 0.25], # Move the robot to a before place pose
        [0.3, 0.0, 0.3] # Move the robot to a neutral pose
    ]
    positions_box4[0][:2]=node.box4_pose[:2]
    positions_box4[1][:2]=node.box4_pose[:2]
    positions_box4[2][:2]=node.box4_pose[:2]
    positions_box4[3][:2]=node.box1_pose[:2]
    positions_box4[4][:2]=node.box1_pose[:2]

    positions_box5 = [
        [0.45, -0.4, 0.2], # Move the robot to a before place pose
        [0.45, -0.4, 0.025], # Pick up the object
        [0.45, -0.4, 0.3], # Move the robot to a before place pose
        [-0.5, 0.25, 0.225], # Place the object
        [-0.5, 0.25, 0.3], # Move the robot to a before place pose
        [0.3, 0.0, 0.3] # Move the robot to a neutral pose
    ]
    positions_box5[0][:2]=node.box5_pose[:2]
    positions_box5[1][:2]=node.box5_pose[:2]
    positions_box5[2][:2]=node.box5_pose[:2]
    positions_box5[3][:2]=node.box1_pose[:2]
    positions_box5[4][:2]=node.box1_pose[:2]

    positions_box6 = [
        [0.65, -0.35, 0.2], # Move the robot to a before place pose
        [0.65, -0.35, 0.025], # Pick up the object
        [0.65, -0.35, 0.35], # Move the robot to a before place pose
        [-0.5, 0.25, 0.275], # Place the object
        [-0.5, 0.25, 0.35], # Move the robot to a before place pose
        [0.3, 0.0, 0.3] # Move the robot to a neutral pose
    ]
    positions_box6[0][:2]=node.box6_pose[:2]
    positions_box6[1][:2]=node.box6_pose[:2]
    positions_box6[2][:2]=node.box6_pose[:2]
    positions_box6[3][:2]=node.box1_pose[:2]
    positions_box6[4][:2]=node.box1_pose[:2]
    # Loop between the left and right side
    
    while rclpy.ok():
  
        while not node.box2_pose or not node.box3_pose or not node.box4_pose or not node.box5_pose or not node.box6_pose:
            node.get_logger().warn("waiting...")
            time.sleep(1)
            continue

        # 执行 pnp() 任务
        pnp(node, node.moveit2, node.gripper_interface, positions_box2, quat_xyzw)
        pnp(node, node.moveit2, node.gripper_interface, positions_box3, quat_xyzw)
        pnp(node, node.moveit2, node.gripper_interface, positions_box4, quat_xyzw)
        pnp(node, node.moveit2, node.gripper_interface, positions_box5, quat_xyzw)
        pnp(node, node.moveit2, node.gripper_interface, positions_box6, quat_xyzw)

        break  # 任务完成后退出 while

        rclpy.shutdown()
        executor_thread.join()
        exit(0)
