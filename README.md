# 🦾 FR3 Box Stacking Simulation (ROS2 + MoveIt2 + Gazebo)

This project implements an autonomous box-stacking system using the Franka Emika FR3 robot in a ROS2 + Gazebo simulation. The robot uses live pose data from Gazebo, plans motion using MoveIt2, and stacks multiple boxes without any hardcoded coordinates. Developed as coursework for the Robotics Foundations module at the University of Glasgow.

## 🎯 Overview

- 🧠 Fully autonomous stacking of 5+ boxes  
- 📡 Reads real-time box positions via Gazebo pose topics  
- 🤖 Uses MoveIt2 for arm motion planning and gripper control  
- 🧱 Each box is grasped, lifted, and stacked at a predefined goal zone  

## 🔧 Technical Highlights

- ROS2 node `pnp_loop.py` using `rclpy`
- Motion planning via `pymoveit2`
- Gripper control via service calls
- Gazebo-ROS2 bridge (`ros_gz_bridge`) to subscribe to `/model/boxX/pose`
- Launch script sets up FR3, controllers, Gazebo world, and bridge
- FSM-based execution: pick → move → stack → repeat


## 📦 Dependencies

- ROS2 Foxy
- MoveIt2 + `pymoveit2`
- `fr3_moveit_config`
- `franka_description`
- `ros_gz_bridge` (for Gazebo <-> ROS2 communication)
