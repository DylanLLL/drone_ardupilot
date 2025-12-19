#!/usr/bin/env python3
"""
Flight Controller Node
Handles high-level flight commands and interfaces with ArduPilot via MAVROS
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import State, PositionTarget, AttitudeTarget
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL
from std_msgs.msg import String
import numpy as np
from enum import Enum
from typing import Optional


class FlightMode(Enum):
    """Flight modes for the drone"""
    MANUAL = "MANUAL"
    STABILIZE = "STABILIZE"
    GUIDED = "GUIDED"
    AUTO = "AUTO"
    LOITER = "LOITER"
    RTL = "RTL"
    LAND = "LAND"


class FlightControllerNode(Node):
    """
    High-level flight controller for autonomous navigation
    """
    
    def __init__(self):
        super().__init__('flight_controller_node')
        
        # Declare parameters
        self.declare_parameter('default_altitude', 1.5)
        self.declare_parameter('takeoff_altitude', 1.5)
        self.declare_parameter('max_velocity', 1.0)
        self.declare_parameter('position_tolerance', 0.15)
        self.declare_parameter('control_rate', 20.0)
        
        # Get parameters
        self.default_altitude = self.get_parameter('default_altitude').value
        self.takeoff_altitude = self.get_parameter('takeoff_altitude').value
        self.max_velocity = self.get_parameter('max_velocity').value
        self.position_tolerance = self.get_parameter('position_tolerance').value
        control_rate = self.get_parameter('control_rate').value
        
        # State variables
        self.mavros_state: Optional[State] = None
        self.current_pose: Optional[PoseStamped] = None
        self.target_pose: Optional[PoseStamped] = None
        self.is_armed = False
        self.current_mode = ""
        
        # Subscribers
        self.state_sub = self.create_subscription(
            State,
            '/mavros/state',
            self.state_callback,
            10
        )
        
        self.local_position_sub = self.create_subscription(
            PoseStamped,
            '/drone/local_position',
            self.local_position_callback,
            10
        )
        
        self.command_sub = self.create_subscription(
            String,
            '/drone/command',
            self.command_callback,
            10
        )
        
        # Publishers
        self.setpoint_position_pub = self.create_publisher(
            PoseStamped,
            '/mavros/setpoint_position/local',
            10
        )
        
        self.setpoint_velocity_pub = self.create_publisher(
            TwistStamped,
            '/mavros/setpoint_velocity/cmd_vel',
            10
        )
        
        # Service clients
        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')
        self.takeoff_client = self.create_client(CommandTOL, '/mavros/cmd/takeoff')
        self.land_client = self.create_client(CommandTOL, '/mavros/cmd/land')
        
        # Control timer
        self.control_timer = self.create_timer(
            1.0 / control_rate,
            self.control_loop
        )
        
        # Wait for MAVROS connection
        self.get_logger().info('Waiting for MAVROS connection...')
        self._wait_for_mavros()
        
        self.get_logger().info('Flight Controller Node initialized')
    
    def _wait_for_mavros(self, timeout_sec=10.0):
        """Wait for MAVROS to connect to flight controller"""
        rate = self.create_rate(10)  # 10 Hz
        start_time = self.get_clock().now()
        
        while rclpy.ok():
            if self.mavros_state is not None and self.mavros_state.connected:
                self.get_logger().info('MAVROS connected!')
                break
            
            if (self.get_clock().now() - start_time).nanoseconds / 1e9 > timeout_sec:
                self.get_logger().warn('MAVROS connection timeout')
                break
            
            rate.sleep()
    
    def state_callback(self, msg: State):
        """Callback for MAVROS state"""
        self.mavros_state = msg
        self.is_armed = msg.armed
        self.current_mode = msg.mode
    
    def local_position_callback(self, msg: PoseStamped):
        """Callback for drone local position"""
        self.current_pose = msg
    
    def command_callback(self, msg: String):
        """Callback for high-level commands"""
        command = msg.data.lower()
        
        if command == 'arm':
            self.arm()
        elif command == 'disarm':
            self.disarm()
        elif command == 'takeoff':
            self.takeoff(self.takeoff_altitude)
        elif command == 'land':
            self.land()
        elif command.startswith('goto'):
            # Parse goto command: "goto x y z"
            try:
                parts = command.split()
                x, y, z = float(parts[1]), float(parts[2]), float(parts[3])
                self.goto_position(x, y, z)
            except (IndexError, ValueError):
                self.get_logger().error('Invalid goto command format. Use: goto x y z')
        else:
            self.get_logger().warn(f'Unknown command: {command}')
    
    def control_loop(self):
        """Main control loop"""
        if self.target_pose is not None and self.is_armed:
            # Publish setpoint
            self.setpoint_position_pub.publish(self.target_pose)
            
            # Check if target reached
            if self.current_pose is not None:
                distance = self._calculate_distance(self.current_pose, self.target_pose)
                if distance < self.position_tolerance:
                    self.get_logger().info(
                        f'Target reached (distance: {distance:.3f}m)',
                        throttle_duration_sec=2.0
                    )
    
    def arm(self):
        """Arm the drone"""
        if not self.is_armed:
            self.get_logger().info('Arming drone...')
            req = CommandBool.Request()
            req.value = True
            
            future = self.arming_client.call_async(req)
            future.add_done_callback(self._arm_callback)
    
    def _arm_callback(self, future):
        """Callback for arm service"""
        try:
            response = future.result()
            if response.success:
                self.get_logger().info('Drone armed successfully')
            else:
                self.get_logger().error('Failed to arm drone')
        except Exception as e:
            self.get_logger().error(f'Arm service call failed: {str(e)}')
    
    def disarm(self):
        """Disarm the drone"""
        if self.is_armed:
            self.get_logger().info('Disarming drone...')
            req = CommandBool.Request()
            req.value = False
            
            future = self.arming_client.call_async(req)
            future.add_done_callback(self._disarm_callback)
    
    def _disarm_callback(self, future):
        """Callback for disarm service"""
        try:
            response = future.result()
            if response.success:
                self.get_logger().info('Drone disarmed successfully')
            else:
                self.get_logger().error('Failed to disarm drone')
        except Exception as e:
            self.get_logger().error(f'Disarm service call failed: {str(e)}')
    
    def set_mode(self, mode: str):
        """Set flight mode"""
        self.get_logger().info(f'Setting mode to {mode}...')
        req = SetMode.Request()
        req.custom_mode = mode
        
        future = self.set_mode_client.call_async(req)
        future.add_done_callback(lambda f: self._mode_callback(f, mode))
    
    def _mode_callback(self, future, mode):
        """Callback for set mode service"""
        try:
            response = future.result()
            if response.mode_sent:
                self.get_logger().info(f'Mode set to {mode}')
            else:
                self.get_logger().error(f'Failed to set mode to {mode}')
        except Exception as e:
            self.get_logger().error(f'Set mode service call failed: {str(e)}')
    
    def takeoff(self, altitude: float):
        """Execute takeoff to specified altitude"""
        self.get_logger().info(f'Taking off to {altitude}m...')
        
        # First, set to GUIDED mode
        self.set_mode('GUIDED')
        
        # Then arm if not already armed
        if not self.is_armed:
            self.arm()
        
        # Set takeoff target
        req = CommandTOL.Request()
        req.altitude = altitude
        
        future = self.takeoff_client.call_async(req)
        future.add_done_callback(self._takeoff_callback)
    
    def _takeoff_callback(self, future):
        """Callback for takeoff service"""
        try:
            response = future.result()
            if response.success:
                self.get_logger().info('Takeoff command sent')
            else:
                self.get_logger().error('Takeoff command failed')
        except Exception as e:
            self.get_logger().error(f'Takeoff service call failed: {str(e)}')
    
    def land(self):
        """Execute landing"""
        self.get_logger().info('Landing...')
        
        req = CommandTOL.Request()
        
        future = self.land_client.call_async(req)
        future.add_done_callback(self._land_callback)
    
    def _land_callback(self, future):
        """Callback for land service"""
        try:
            response = future.result()
            if response.success:
                self.get_logger().info('Land command sent')
            else:
                self.get_logger().error('Land command failed')
        except Exception as e:
            self.get_logger().error(f'Land service call failed: {str(e)}')
    
    def goto_position(self, x: float, y: float, z: float):
        """Command drone to go to specified position"""
        self.get_logger().info(f'Going to position: [{x:.2f}, {y:.2f}, {z:.2f}]')
        
        # Create target pose
        target = PoseStamped()
        target.header.stamp = self.get_clock().now().to_msg()
        target.header.frame_id = 'map'
        target.pose.position.x = x
        target.pose.position.y = y
        target.pose.position.z = z
        
        # Keep current orientation
        if self.current_pose is not None:
            target.pose.orientation = self.current_pose.pose.orientation
        else:
            # Default orientation (facing forward)
            target.pose.orientation.w = 1.0
        
        self.target_pose = target
    
    def _calculate_distance(self, pose1: PoseStamped, pose2: PoseStamped) -> float:
        """Calculate Euclidean distance between two poses"""
        dx = pose1.pose.position.x - pose2.pose.position.x
        dy = pose1.pose.position.y - pose2.pose.position.y
        dz = pose1.pose.position.z - pose2.pose.position.z
        return np.sqrt(dx**2 + dy**2 + dz**2)


def main(args=None):
    rclpy.init(args=args)
    node = FlightControllerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
