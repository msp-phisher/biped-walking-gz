# ROS2 Biped Walking Simulation

A custom 10-DOF biped robot simulated in ROS2 Humble with Ignition Gazebo using `ros2_control`.

This repository demonstrates a basic quasi-static walking gait and provides:
- URDF model with Xacro
- Joint controllers configured via ros2_control
- Python script for simple alternating leg motion
- Simulation launch files

---

## Features

- Custom planar biped with 5 DOF per leg
- Gazebo simulation
- JointTrajectoryController position control
- Alternating swing walking demo

---
## Requirements

- ROS2 Humble
- Ignition Gazebo
- ros2_control + joint_trajectory_controller
- Python3

---

##  Installation

```bash
cd ~/ros2_ws/src
git clone https://github.com/msp-phisher/biped-walking-gz.git
cd ~/ros2_ws
colcon build
source install/setup.bash
ros2 launch biped_description gazebo.launch.py

In second Terminal
Build and python3 walk.py

<img width="1852" height="1046" alt="image" src="https://github.com/user-attachments/assets/edcd5505-60d0-423f-83d0-0d9c58e1aa55" />

## About Walking

This project currently uses a quasi-static open-loop walking gait:
Lifts one leg at a time
Slow leg swing
No dynamic balance controller
Designed for learning & simulation

## Status & Limitations

No lateral balance (hip_roll control)
No IMU feedback
Not suitable for rough terrain
Future: ZMP, LIPM, stable balance

