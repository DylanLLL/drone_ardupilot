#!/usr/bin/env python3
"""
Main launch file for warehouse drone navigation system
Launches all core nodes
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Package directory
    pkg_share = FindPackageShare('warehouse_drone_nav')
    
    # Configuration files
    aruco_config = PathJoinSubstitution([pkg_share, 'config', 'aruco_detector.yaml'])
    position_config = PathJoinSubstitution([pkg_share, 'config', 'position_estimator.yaml'])
    flight_config = PathJoinSubstitution([pkg_share, 'config', 'flight_controller.yaml'])
    mission_config = PathJoinSubstitution([pkg_share, 'config', 'mission_manager.yaml'])
    
    # Declare launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )
    
    # ArUco Detector Node
    aruco_detector_node = Node(
        package='warehouse_drone_nav',
        executable='aruco_detector_node.py',
        name='aruco_detector',
        output='screen',
        parameters=[
            aruco_config,
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ]
    )
    
    # Position Estimator Node
    position_estimator_node = Node(
        package='warehouse_drone_nav',
        executable='position_estimator_node.py',
        name='position_estimator',
        output='screen',
        parameters=[
            position_config,
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ]
    )
    
    # Flight Controller Node
    flight_controller_node = Node(
        package='warehouse_drone_nav',
        executable='flight_controller_node.py',
        name='flight_controller',
        output='screen',
        parameters=[
            flight_config,
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ]
    )
    
    # Mission Manager Node
    mission_manager_node = Node(
        package='warehouse_drone_nav',
        executable='mission_manager_node.py',
        name='mission_manager',
        output='screen',
        parameters=[
            mission_config,
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ]
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        aruco_detector_node,
        position_estimator_node,
        flight_controller_node,
        mission_manager_node,
    ])
