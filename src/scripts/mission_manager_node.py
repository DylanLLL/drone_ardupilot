#!/usr/bin/env python3
"""
Mission Manager Node
Coordinates autonomous navigation missions between ArUco markers
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
import numpy as np
from enum import Enum
from typing import List, Optional, Tuple
import yaml


class MissionState(Enum):
    """Mission states"""
    IDLE = "IDLE"
    INITIALIZING = "INITIALIZING"
    TAKING_OFF = "TAKING_OFF"
    NAVIGATING = "NAVIGATING"
    HOVERING = "HOVERING"
    LANDING = "LANDING"
    COMPLETED = "COMPLETED"
    EMERGENCY = "EMERGENCY"


class MissionManagerNode(Node):
    """
    Manages autonomous navigation missions
    """
    
    def __init__(self):
        super().__init__('mission_manager_node')
        
        # Declare parameters
        self.declare_parameter('mission_file', '')
        self.declare_parameter('auto_start', False)
        self.declare_parameter('waypoint_tolerance', 0.2)
        
        # Get parameters
        mission_file = self.get_parameter('mission_file').value
        self.auto_start = self.get_parameter('auto_start').value
        self.waypoint_tolerance = self.get_parameter('waypoint_tolerance').value
        
        # Mission state
        self.state = MissionState.IDLE
        self.waypoints: List[Tuple[float, float, float]] = []
        self.current_waypoint_idx = 0
        self.current_pose: Optional[PoseStamped] = None
        
        # Load mission if provided
        if mission_file:
            self._load_mission(mission_file)
        
        # Subscribers
        self.local_position_sub = self.create_subscription(
            PoseStamped,
            '/drone/local_position',
            self.local_position_callback,
            10
        )
        
        self.mission_command_sub = self.create_subscription(
            String,
            '/mission/command',
            self.mission_command_callback,
            10
        )
        
        # Publishers
        self.flight_command_pub = self.create_publisher(
            String,
            '/drone/command',
            10
        )
        
        self.mission_status_pub = self.create_publisher(
            String,
            '/mission/status',
            10
        )
        
        # Timer for mission monitoring
        self.mission_timer = self.create_timer(0.5, self.mission_loop)
        
        self.get_logger().info('Mission Manager Node initialized')
        self.get_logger().info(f'Current state: {self.state.value}')
        
        if self.auto_start and self.waypoints:
            self.get_logger().info('Auto-starting mission...')
            self.start_mission()
    
    def _load_mission(self, filepath: str):
        """Load mission waypoints from YAML file"""
        try:
            with open(filepath, 'r') as f:
                data = yaml.safe_load(f)
            
            if 'waypoints' in data:
                for wp in data['waypoints']:
                    waypoint = (
                        float(wp['x']),
                        float(wp['y']),
                        float(wp['z'])
                    )
                    self.waypoints.append(waypoint)
                
                self.get_logger().info(f'Loaded {len(self.waypoints)} waypoints from {filepath}')
            else:
                self.get_logger().error('Invalid mission file format')
        
        except Exception as e:
            self.get_logger().error(f'Failed to load mission: {str(e)}')
    
    def local_position_callback(self, msg: PoseStamped):
        """Callback for drone local position"""
        self.current_pose = msg
    
    def mission_command_callback(self, msg: String):
        """Callback for mission commands"""
        command = msg.data.lower()
        
        if command == 'start':
            self.start_mission()
        elif command == 'pause':
            self.pause_mission()
        elif command == 'resume':
            self.resume_mission()
        elif command == 'abort':
            self.abort_mission()
        elif command == 'land':
            self.emergency_land()
        else:
            self.get_logger().warn(f'Unknown mission command: {command}')
    
    def start_mission(self):
        """Start the mission"""
        if not self.waypoints:
            self.get_logger().error('No waypoints loaded. Cannot start mission.')
            return
        
        self.get_logger().info('Starting mission...')
        self.state = MissionState.INITIALIZING
        self.current_waypoint_idx = 0
        
        # Send takeoff command
        self._send_flight_command('takeoff')
        self.state = MissionState.TAKING_OFF
        
        self._publish_status('Mission started - taking off')
    
    def pause_mission(self):
        """Pause the mission"""
        if self.state == MissionState.NAVIGATING:
            self.get_logger().info('Pausing mission...')
            self.state = MissionState.HOVERING
            self._publish_status('Mission paused')
    
    def resume_mission(self):
        """Resume the mission"""
        if self.state == MissionState.HOVERING:
            self.get_logger().info('Resuming mission...')
            self.state = MissionState.NAVIGATING
            self._publish_status('Mission resumed')
    
    def abort_mission(self):
        """Abort the mission and land"""
        self.get_logger().warn('Aborting mission!')
        self.state = MissionState.LANDING
        self._send_flight_command('land')
        self._publish_status('Mission aborted - landing')
    
    def emergency_land(self):
        """Emergency landing"""
        self.get_logger().error('EMERGENCY LAND!')
        self.state = MissionState.EMERGENCY
        self._send_flight_command('land')
        self._publish_status('EMERGENCY - landing')
    
    def mission_loop(self):
        """Main mission state machine"""
        if self.state == MissionState.TAKING_OFF:
            self._handle_takeoff()
        elif self.state == MissionState.NAVIGATING:
            self._handle_navigation()
        elif self.state == MissionState.LANDING:
            self._handle_landing()
    
    def _handle_takeoff(self):
        """Handle takeoff state"""
        if self.current_pose is None:
            return
        
        # Check if takeoff altitude reached
        target_altitude = 1.5  # meters
        current_altitude = self.current_pose.pose.position.z
        
        if abs(current_altitude - target_altitude) < 0.3:
            self.get_logger().info('Takeoff complete, starting navigation')
            self.state = MissionState.NAVIGATING
            self._navigate_to_next_waypoint()
    
    def _handle_navigation(self):
        """Handle navigation state"""
        if self.current_pose is None:
            return
        
        # Check if current waypoint reached
        if self.current_waypoint_idx < len(self.waypoints):
            target = self.waypoints[self.current_waypoint_idx]
            distance = self._calculate_distance_to_point(target)
            
            if distance < self.waypoint_tolerance:
                self.get_logger().info(
                    f'Waypoint {self.current_waypoint_idx + 1}/{len(self.waypoints)} reached'
                )
                self.current_waypoint_idx += 1
                
                if self.current_waypoint_idx < len(self.waypoints):
                    self._navigate_to_next_waypoint()
                else:
                    self.get_logger().info('All waypoints completed, landing...')
                    self.state = MissionState.LANDING
                    self._send_flight_command('land')
    
    def _handle_landing(self):
        """Handle landing state"""
        if self.current_pose is None:
            return
        
        # Check if landed (altitude near zero)
        if self.current_pose.pose.position.z < 0.1:
            self.get_logger().info('Landing complete')
            self.state = MissionState.COMPLETED
            self._publish_status('Mission completed')
            
            # Send disarm command
            self._send_flight_command('disarm')
    
    def _navigate_to_next_waypoint(self):
        """Send command to navigate to next waypoint"""
        if self.current_waypoint_idx >= len(self.waypoints):
            return
        
        waypoint = self.waypoints[self.current_waypoint_idx]
        self.get_logger().info(
            f'Navigating to waypoint {self.current_waypoint_idx + 1}/{len(self.waypoints)}: '
            f'[{waypoint[0]:.2f}, {waypoint[1]:.2f}, {waypoint[2]:.2f}]'
        )
        
        command = f'goto {waypoint[0]} {waypoint[1]} {waypoint[2]}'
        self._send_flight_command(command)
        
        self._publish_status(
            f'Navigating to waypoint {self.current_waypoint_idx + 1}/{len(self.waypoints)}'
        )
    
    def _calculate_distance_to_point(self, point: Tuple[float, float, float]) -> float:
        """Calculate distance from current position to target point"""
        if self.current_pose is None:
            return float('inf')
        
        dx = self.current_pose.pose.position.x - point[0]
        dy = self.current_pose.pose.position.y - point[1]
        dz = self.current_pose.pose.position.z - point[2]
        
        return np.sqrt(dx**2 + dy**2 + dz**2)
    
    def _send_flight_command(self, command: str):
        """Send command to flight controller"""
        msg = String()
        msg.data = command
        self.flight_command_pub.publish(msg)
    
    def _publish_status(self, status: str):
        """Publish mission status"""
        msg = String()
        msg.data = f'[{self.state.value}] {status}'
        self.mission_status_pub.publish(msg)
        self.get_logger().info(status)


def main(args=None):
    rclpy.init(args=args)
    node = MissionManagerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
