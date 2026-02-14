# Biped Walking Robot in ROS2 (Ignition Gazebo)

A 10-DOF planar biped robot developed in ROS2 (Humble) using `ros2_control` and Ignition Gazebo for physics simulation.  
This project demonstrates basic joint-space walking through trajectory control without external balance controllers.

---

## Overview

This repository contains:

- Custom URDF/Xacro biped model
- ros2_control hardware interface configuration
- JointTrajectoryController based motion execution
- Ignition Gazebo simulation setup
- Python walking script for alternating swing motion

The robot consists of:

- 2 legs
- 5 DOF per leg:
  - Hip roll
  - Hip pitch
  - Knee
  - Ankle roll
  - Ankle pitch

Total: **10 actuated joints**

---

## System Architecture

ROS2 Node (walk.py)
↓
JointTrajectoryController
↓
ros2_control (IgnitionSystem)
↓
Ignition Gazebo Physics Engine
↓
URDF Model


Control mode:
- Position interface via JointTrajectoryController
- Open-loop joint space walking
- No ZMP / MPC / LQR balance controller

---

## Dependencies

- Ubuntu 22.04
- ROS2 Humble
- Ignition Gazebo (Fortress / compatible)
- ros2_control
- joint_trajectory_controller

Install required packages:

```bash
sudo apt install ros-humble-ros2-control \
                 ros-humble-ros2-controllers \
                 ros-humble-joint-state-publisher-gui \
                 ros-humble-robot-state-publisher
