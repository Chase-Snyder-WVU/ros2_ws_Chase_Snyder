# Square-Path Mobile Robot Simulation (ROS 2 Humble)
**Author:** Chase Snyder
**Course:** ROBE 313 - West Virginia University
**Assignment:** Homework 3 - ROS 2 Multi-Package System

## Project Overview
This project implements a ROS 2 Humble simulation for a mobile robot that drives in a **2m x 2m square** using proportional linear control in an Ubuntu 22.04 WSL environment.
> **Note:** 
> This implementation uses **only linear x and y velocity** with no turning or angular models.

## Main Features
- C++ odometry node that integrats velocity to estimate robot pose and publishes robot pose with `/tf` broadcast.
- Python controller node using TF2 feeback and a state machine to drive the square path by publishing velocity commands.
- Custom `.srv` service to reset robot pose.
- URDF robot model and visualization in RViz.

## Workspace Structure
ros2_ws_Chase Snyder/
    src/
        custom_interfaces/
        robot_simulator_cpp/
        robot_simulator_py/
        robot_bringup/

## Package Summary
- **custom_interfaces** holds `ResetPosition.srv`
- **robot_simulator_cpp** publishes `/tf`, integrates odometry, and provides reset service
- **robot_simulator_py** TF2 controller state machine that publishes `/cmd_vel` for square path.
- **robot_bringup** holds the launch file, URDF, and RViz config

## Requirements
- ROS 2 Humble  (Ubuntu 22.04 / WSL)
- `colcon`
- `ament`
- `rviz2`
- `Python3`
- `rclpy`
- `C++17`
- `rclcpp`

## Build Instructions

### 1. Clone Workspace
Open a terminal and input the following command.
```bash
git clone https://github.com/Chase-Snyder-WVU/ros2_ws_Chase_Snyder.git
```
### 2. Source ROS 2
Source Ros 2 on your system.
```bash
source /opt/ros/humble/setup.bash
```
### 3. Build
Source the project folder directory and build it.
```bash
cd ros2_ws_Chase_Snyder2
colcon build
source install/setup.bash
```
### 4. Run Simulation 
From ros2_ws_Chase_Snyder2 run the simulation.
```bash
ros2 launch robot_bringup robot_simulation.launch.py
```
>**WSL Note:**
>WSL fails to render the graphics with the base settings, the following commands will force software rendering though RViz remains unstable. Run them before sending the launch command to the terminal if you have this issue.
```bash
export LIBGL_ALWAYS_SOFTWARE=1
export MESA_GL_VERSION_OVERRIDE=3.3
```
>**RViz Note:** 
> When using RViz, the Topic and Parameter options did not function properly. You may have to directly reference the URDF file path (robot_bringup->urdf) under robot model->robot description in RViz.
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
In a seperate terminal while the project is running. Display the node communication as an rqt_graph.
```bash
rqt_graph
```
### View Transforms
In a seperate terminal while the odometry node, controller node, or the project is running.
```bash
ros2 run tf2_ros tf2_echo odom base_link
```
### Reset Robot Pose Service
In a seperate terminal while the odometry node, controller node, or the project is running.
```bash
ros2 service call /reset_position custom_interfaces/srv ResetPosition "{pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}"
```
## Known Bugs

### "No Frames" Intialization
This bug uncommonly occurs when the project is started, often displaying a failed transform message when it occurs. This is likely because on
start-up the odometry node and controller node fail to see eachother's initialization commands resulting in an idle white robot in RViz,





