# @file robot_simulation.launch.py
#
# @author Chase Snyder
#
# @brief Launch file for co-ordinatin odometry node
#        for robot representation and teleop_twist_keyboard
#        utlizing Rviz and Urdf. 

from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():

    #Get the local packages in the folder
    pkg_share = FindPackageShare('robot_bringup').find('robot_bringup')
    urdf_path = os.path.join(pkg_share, 'urdf', 'robot.urdf.xml')
    rviz_path = os.path.join(pkg_share, 'rviz', 'simulation.rviz')

    #Read URDF text for robot_state_publisher param
    with open(urdf_path, 'r') as f:
        robot_desc = f.read()

    #Build the neccessary Nodes
    #All of them display output mostly
    #for debugging but also proving 
    #project is working.
    return LaunchDescription([
        Node(
            package='robot_simulator_cpp',
            executable='odometry_node',
            name='odometry_node',
            output='screen'
        ),
        #kebooard teleop
        Node(
            package='teleop_twist_keyboard',
            executable='teleop_twist_keyboard',
            name='teleop_twist_keyboard',
            output='screen',
            emulate_tty = True
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_desc}]
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_path],
            output='screen'
        )
        
    ])
    