Square-Path Mobile Robot Simulation (ROS 2 Humble)
Author: Chase Snyder
Course: ROBE 313 - West Virginia University
Assignment: Homework 3 - ROS 2 Multi-Package System

Project Overview
This project implements a ROS2 Humble simulation for a mobile robot that drives in a 2m x 2m square using proportional linear control in an Ubuntu 22.04 WSL enviroment.

Main Features
- C++ odometry node for integrating velocity commands to estimate robot pose and publish said poses.
- Python controller node using tf2 feeback and a state machine to follow the square path and send velocity commands.
- Custom .srv Service to reset pose.
- URDF robot model and visualization in RViz.

Workspace Structure
ros2_ws_Chase Snyder/
    src/
        custom_interfaces/
        robot_simulator_cpp/
        robot_simulator_py/
        robot_bringup/

Package Summary
- custom_interfaces hold ResetPosition.srv .
- robot_simulator_cpp publishes /tf, integrates odometry, and provides reset service.
- robot_simulator_py is a controller that responds to tf2 and publishes /cmd_vel to follow a square path with a state machine.
- robot_bringup holds the launch file, URDF, and RViz configuration.

Requirments
- ROS 2 Humble  (Ubuntu 22.04 / WSL)
- colcon
- ament
- RViz2
- Python3
- rclpy
- C++17
- rclcpp

Build Instructions
1. Clone Workspace

2. Source ROS2
    source /opt/ros/humble/setup.bash

3. Build
    cd ros2_ws_Chase_Snyder2
    colcon build
    source install/setup.bash

4. Run Simulation (from ros2_ws_Chase_Snyder2)
    ros2 launch robot_bringup robot_simulation.launch.py

Useful Commands
    Run Odometry Node
    ros2 run robot_simulator_cpp odometry_node
    Run Only Controller
    ros2 run robot_simulator_py controller_node
    Inspect Graph
    rqt_graph
    View Transforms
    ros2 run tf2_tolls view_frames.py
    Reset Robot Pose Service
    ros2 service call /reset_position custom_interfaces/srv ResetPosition "{pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}"


