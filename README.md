# ROS2 Biped Walking Simulation

A custom 10-DOF biped robot simulated in ROS2 Humble with Ignition Gazebo using `ros2_control`.

This repository demonstrates a basic quasi-static walking gait and provides:
- URDF model with Xacro
- Joint controllers configured via ros2_control
- Python script for simple alternating leg motion
- Simulation launch files
  
## Simulation Preview

![Biped Simulation](docs/images/Screenshot%20from%202026-02-15%2001-42-39.png)


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

## In second Terminal
python3 walk.py



