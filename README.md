# Square-Path Mobile Robot Simulation (ROS 2 Humble)
**Author:** Chase Snyder
**Course:** ROBE 313 - West Virginia University
**Assignment:** Homework 3 - ROS 2 Multi-Package System

---

## Project Overview
This project implements a ROS 2 Humble simulation for a mobile robot that drives in a **2m x 2m square** using proportional linear control in an Ubuntu 22.04 WSL environment.
> **Note:** 
> This implementation uses **only linear x and y velocity** with no turning or angular models.

---

## Main Features
- C++ odometry node that integrats velocity to estimate robot pose and publishes robot pose with `/tf` broadcast.
- Python controller node using TF2 feeback and a state machine to drive the square path by publishing velocity commands.
- Custom `.srv` service to reset robot pose.
- URDF robot model and visualization in RViz.

---

## Workspace Structure
ros2_ws_Chase Snyder/
    src/
        custom_interfaces/
        robot_simulator_cpp/
        robot_simulator_py/
        robot_bringup/

---

## Package Summary
- **custom_interfaces** holds `ResetPosition.srv`
- **robot_simulator_cpp** publishes `/tf`, integrates odometry, and provides reset service
- **robot_simulator_py** TF2 controller state machine that publishes `/cmd_vel` for square path.
- **robot_bringup** holds the launch file, URDF, and RViz config

---

## Requirements
- ROS 2 Humble  (Ubuntu 22.04 / WSL)
- `colcon`
- `ament`
- `rviz2`
- `Python3`
- `rclpy`
- `C++17`
- `rclcpp`

---

## Build Instructions

### 1. Clone Workspace

### 2. Source ROS 2
```bash
source /opt/ros/humble/setup.bash
```
### 3. Build
```bash
cd ros2_ws_Chase_Snyder2
colcon build
source install/setup.bash
```
### 4. Run Simulation 
from ros2_ws_Chase_Snyder2
```bash
ros2 launch robot_bringup robot_simulation.launch.py
```

---

## Useful Commands

### Run Odometry Node
```bash
ros2 run robot_simulator_cpp odometry_node
```
### Run Only Controller
```bash
ros2 run robot_simulator_py controller_node
```
### Inspect Graph
```bash
rqt_graph
```
### View Transforms
```bash
ros2 run tf2_tools view_frames.py
```
### Reset Robot Pose Service
```bash
ros2 service call /reset_position custom_interfaces/srv ResetPosition "{pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}"
```


