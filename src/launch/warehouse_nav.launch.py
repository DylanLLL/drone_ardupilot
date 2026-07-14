#!/usr/bin/env python3
"""
Main launch file for warehouse drone navigation system
Launches all core nodes
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
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
    recorder_config = PathJoinSubstitution([pkg_share, 'config', 'flight_test_recorder.yaml'])
    
    # Declare launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )

    record_test_arg = DeclareLaunchArgument(
        'record_test',
        default_value='false',
        description='Enable flight-test recorder logs'
    )

    recorder_session_arg = DeclareLaunchArgument(
        'recorder_session',
        default_value='',
        description='Optional flight-test recorder session folder name'
    )

    # Camera profile: selects config/camera_<profile>.yaml for the usb_cam node.
    # gopro_hero4 = GoPro HERO4 via HDMI-USB capture card (default video source)
    # c270        = legacy Logitech C270 webcam
    # none        = don't launch a camera (started externally, e.g. by hand)
    camera_arg = DeclareLaunchArgument(
        'camera',
        default_value='gopro_hero4',
        description='Camera profile to launch: gopro_hero4, c270, or none'
    )

    camera_profile = LaunchConfiguration('camera')

    # Camera Node (usb_cam works for both the C270 and the GoPro's UVC capture
    # card; only the profile yaml differs). Publishes /camera/image_raw and
    # /camera/camera_info, which aruco_detector consumes.
    camera_node = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='usb_cam',
        namespace='camera',
        output='screen',
        condition=IfCondition(PythonExpression(["'", camera_profile, "' != 'none'"])),
        parameters=[
            PathJoinSubstitution([
                pkg_share, 'config', ['camera_', camera_profile, '.yaml']
            ])
        ]
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

    flight_test_recorder_node = Node(
        package='warehouse_drone_nav',
        executable='flight_test_recorder_node.py',
        name='flight_test_recorder',
        output='screen',
        condition=IfCondition(LaunchConfiguration('record_test')),
        parameters=[
            recorder_config,
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'session_name': LaunchConfiguration('recorder_session')
            }
        ]
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        record_test_arg,
        recorder_session_arg,
        camera_arg,
        camera_node,
        aruco_detector_node,
        position_estimator_node,
        flight_controller_node,
        mission_manager_node,
        flight_test_recorder_node,
    ])
