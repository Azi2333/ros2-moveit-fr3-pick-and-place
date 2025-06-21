# Copyright (c) 2024 Franka Robotics GmbH
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# Run with ros2 launch franka_gazebo_bringup gazebo_moveit.launch.py

import os
import xacro

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch import LaunchContext, LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import  LaunchConfiguration
from launch.actions import AppendEnvironmentVariable
from launch_ros.actions import Node
def get_robot_description(context: LaunchContext, arm_id, load_gripper, franka_hand):
    arm_id_str = context.perform_substitution(arm_id)
    load_gripper_str = context.perform_substitution(load_gripper)
    franka_hand_str = context.perform_substitution(franka_hand)

    franka_xacro_file = os.path.join(
        get_package_share_directory('franka_description'),
        'robots',
        arm_id_str,
        arm_id_str + '.urdf.xacro'
    )

    robot_description_config = xacro.process_file(
        franka_xacro_file,
        mappings={
            'arm_id': arm_id_str,
            'hand': load_gripper_str,
            'ros2_control': 'true',
            'gazebo': 'true',
            'ee_id': franka_hand_str
        }
    )

    robot_description = {'robot_description': robot_description_config.toxml()}

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[
            {"use_sim_time": True},
            robot_description,
        ]
    )

    return [robot_state_publisher]


def prepare_launch_description():
    # Configure ROS nodes for launch
    load_gripper_name = 'load_gripper'
    franka_hand_name = 'franka_hand'
    arm_id_name = 'arm_id'
    world_type_name = 'world_type'

    load_gripper = LaunchConfiguration(load_gripper_name)
    franka_hand = LaunchConfiguration(franka_hand_name)
    arm_id = LaunchConfiguration(arm_id_name)
    world_type = LaunchConfiguration(world_type_name)

    load_gripper_launch_argument = DeclareLaunchArgument(
            load_gripper_name,
            default_value='true',
            description='true/false for activating the gripper')
    franka_hand_launch_argument = DeclareLaunchArgument(
            franka_hand_name,
            default_value='franka_hand',
            description='Default value: franka_hand')
    arm_id_launch_argument = DeclareLaunchArgument(
            arm_id_name,
            default_value='fr3',
            description='Available values: fr3')
    world_type_argument = DeclareLaunchArgument(
            world_type_name,
            default_value=TextSubstitution(text='coursework_rf.sdf'),
            description='Available values: empty, target')

    # Get robot description
    robot_state_publisher = OpaqueFunction(
        function=get_robot_description,
        args=[arm_id, load_gripper, franka_hand])

    # Gazebo Sim
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_ros_franka_desc = get_package_share_directory('lab5_gz_environment')
    gazebo_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': pkg_ros_franka_desc+'/launch/coursework_rf.sdf -r', }.items(),
    )

    # Spawn
    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', '/robot_description'],
        output='screen',
    )

    # Visualize in RViz
    # rviz_file = os.path.join(get_package_share_directory('franka_description'), 'rviz',
    #                          'visualize_franka.rviz')
    # rviz = Node(package='rviz2',
    #          executable='rviz2',
    #          name='rviz2',
    #          arguments=['--display-config', rviz_file, '-f', 'world'],
    # )

    load_joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
                'joint_state_broadcaster'],
        output='screen'
    )

    # Make the /clock topic available in ROS
    gz_sim_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/model/box1/pose@geometry_msgs/msg/PoseStamped[gz.msgs.Pose",
            "/model/box2/pose@geometry_msgs/msg/PoseStamped[gz.msgs.Pose",
            "/model/box3/pose@geometry_msgs/msg/PoseStamped[gz.msgs.Pose",
            "/model/box4/pose@geometry_msgs/msg/PoseStamped[gz.msgs.Pose",
            "/model/box5/pose@geometry_msgs/msg/PoseStamped[gz.msgs.Pose",
            "/model/box6/pose@geometry_msgs/msg/PoseStamped[gz.msgs.Pose"
            # "/model/sphere/pose@geometry_msgs/msg/PoseStamped[gz.msgs.Pose"
        ],
        output="screen",
    )

    return LaunchDescription([
        load_gripper_launch_argument,
        franka_hand_launch_argument,
        arm_id_launch_argument,
        # world_type_argument,
        gazebo_world,
        robot_state_publisher,
        # rviz,
        gz_sim_bridge,
        spawn,
        RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=spawn,
                    on_exit=[load_joint_state_broadcaster],
                )
        ),
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            parameters=[
                {'source_list': ['joint_states'],
                 'rate': 30}],
        ),
    ])

def generate_launch_description():
    launch_description = prepare_launch_description()

    set_env_vars_resources = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        os.path.join(get_package_share_directory('franka_description')))

    launch_description.add_action(set_env_vars_resources)

    return launch_description
