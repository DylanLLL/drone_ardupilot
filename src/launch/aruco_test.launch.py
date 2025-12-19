#!/usr/bin/env python3
"""
Launch file for testing ArUco detection only
Useful for calibration and testing camera setup
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Package directory
    pkg_share = FindPackageShare('warehouse_drone_nav')
    
    # Configuration file
    aruco_config = PathJoinSubstitution([pkg_share, 'config', 'aruco_detector.yaml'])
    
    # Declare launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )
    
    visualize_arg = DeclareLaunchArgument(
        'visualize',
        default_value='true',
        description='Enable visualization'
    )
    
    # ArUco Detector Node
    aruco_detector_node = Node(
        package='warehouse_drone_nav',
        executable='aruco_detector_node.py',
        name='aruco_detector',
        output='screen',
        parameters=[
            aruco_config,
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'visualize': LaunchConfiguration('visualize')
            }
        ]
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        visualize_arg,
        aruco_detector_node,
    ])
