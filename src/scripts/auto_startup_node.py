#!/usr/bin/env python3
"""
Auto Startup Node
Automates the drone startup sequence:
1. Set mode to GUIDED
2. Takeoff to 50-60cm altitude
3. Position lock automatically engages when ArUco tag is detected (handled by flight_controller_node.py)
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL
from std_msgs.msg import String
import time


class AutoStartupNode(Node):
    """
    Automated startup sequence for drone
    """

    def __init__(self):
        super().__init__('auto_startup')

        # Declare parameters
        self.declare_parameter('target_altitude', 0.55)  # 55cm default (between 50-60cm)
        self.declare_parameter('altitude_tolerance', 0.05)  # 5cm tolerance
        self.declare_parameter('startup_timeout', 30.0)  # 30 seconds timeout

        # Get parameters
        self.target_altitude = self.get_parameter('target_altitude').value
        self.altitude_tolerance = self.get_parameter('altitude_tolerance').value
        self.startup_timeout = self.get_parameter('startup_timeout').value

        # State variables
        self.mavros_state = None
        self.current_pose = None
        self.startup_complete = False
        self.startup_start_time = None
        self.sequence_step = 0  # Track which step we're on

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

        # Publishers
        self.setpoint_position_pub = self.create_publisher(
            PoseStamped,
            '/mavros/setpoint_position/local',
            10
        )

        # Service clients
        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')
        self.takeoff_client = self.create_client(CommandTOL, '/mavros/cmd/takeoff')

        # Wait for services to be available
        self.get_logger().info('Waiting for MAVROS services...')
        self.arming_client.wait_for_service(timeout_sec=10.0)
        self.set_mode_client.wait_for_service(timeout_sec=10.0)
        self.takeoff_client.wait_for_service(timeout_sec=10.0)
        self.get_logger().info('MAVROS services available')

        # Wait a bit for state to be received
        time.sleep(2.0)

        # Start the automated sequence
        self.get_logger().info('Starting automated drone startup sequence...')
        self.startup_start_time = self.get_clock().now()
        self.start_sequence()

        # Create timer to monitor progress
        self.monitor_timer = self.create_timer(0.5, self.monitor_progress)

    def state_callback(self, msg: State):
        """Callback for MAVROS state"""
        self.mavros_state = msg

    def local_position_callback(self, msg: PoseStamped):
        """Callback for drone local position"""
        self.current_pose = msg

    def start_sequence(self):
        """Start the automated startup sequence"""
        self.sequence_step = 1
        self.set_guided_mode()

    def set_guided_mode(self):
        """Step 1: Set mode to GUIDED"""
        self.get_logger().info('Step 1: Setting mode to GUIDED...')
        req = SetMode.Request()
        req.custom_mode = 'GUIDED'

        future = self.set_mode_client.call_async(req)
        future.add_done_callback(self.guided_mode_callback)

    def guided_mode_callback(self, future):
        """Callback for GUIDED mode setting"""
        try:
            response = future.result()
            if response.mode_sent:
                self.get_logger().info('GUIDED mode set successfully')
                # Wait a moment for mode to stabilize, then proceed to arm and takeoff
                time.sleep(1.0)
                self.sequence_step = 2
                self.arm_and_takeoff()
            else:
                self.get_logger().error('Failed to set GUIDED mode')
                self.shutdown_node()
        except Exception as e:
            self.get_logger().error(f'Set mode service call failed: {str(e)}')
            self.shutdown_node()

    def arm_and_takeoff(self):
        """Step 2: Arm the drone and initiate takeoff"""
        self.get_logger().info(f'Step 2: Arming and taking off to {self.target_altitude}m altitude...')

        # Set target position for continuous setpoint publishing
        if self.current_pose is not None:
            target = PoseStamped()
            target.header.stamp = self.get_clock().now().to_msg()
            target.header.frame_id = 'map'
            target.pose.position.x = self.current_pose.pose.position.x
            target.pose.position.y = self.current_pose.pose.position.y
            target.pose.position.z = self.target_altitude
            target.pose.orientation = self.current_pose.pose.orientation

            # Start publishing setpoints
            self.target_pose = target
            self.setpoint_timer = self.create_timer(0.05, self.publish_setpoint)

        # Arm the drone
        is_armed = self.mavros_state is not None and self.mavros_state.armed
        if not is_armed:
            arm_req = CommandBool.Request()
            arm_req.value = True
            arm_future = self.arming_client.call_async(arm_req)
            arm_future.add_done_callback(self.arm_callback)
        else:
            self.get_logger().info('Drone already armed')
            self.initiate_takeoff()

    def publish_setpoint(self):
        """Continuously publish setpoint position"""
        if hasattr(self, 'target_pose'):
            self.setpoint_position_pub.publish(self.target_pose)

    def arm_callback(self, future):
        """Callback for arming"""
        try:
            response = future.result()
            if response.success:
                self.get_logger().info('Drone armed successfully')
                time.sleep(0.5)
                self.initiate_takeoff()
            else:
                self.get_logger().error('Failed to arm drone')
                self.shutdown_node()
        except Exception as e:
            self.get_logger().error(f'Arm service call failed: {str(e)}')
            self.shutdown_node()

    def initiate_takeoff(self):
        """Send takeoff command"""
        takeoff_req = CommandTOL.Request()
        takeoff_req.altitude = self.target_altitude

        takeoff_future = self.takeoff_client.call_async(takeoff_req)
        takeoff_future.add_done_callback(self.takeoff_callback)

    def takeoff_callback(self, future):
        """Callback for takeoff command"""
        try:
            response = future.result()
            if response.success:
                self.get_logger().info('Takeoff command sent successfully')
                self.sequence_step = 3
            else:
                self.get_logger().error('Takeoff command failed')
                self.shutdown_node()
        except Exception as e:
            self.get_logger().error(f'Takeoff service call failed: {str(e)}')
            self.shutdown_node()

    def monitor_progress(self):
        """Monitor the progress of the startup sequence"""
        if self.startup_complete:
            return

        # Check for timeout
        elapsed = (self.get_clock().now() - self.startup_start_time).nanoseconds / 1e9
        if elapsed > self.startup_timeout:
            self.get_logger().error(f'Startup sequence timed out after {self.startup_timeout}s')
            self.shutdown_node()
            return

        # Step 3: Monitor altitude and wait for target
        if self.sequence_step == 3:
            if self.current_pose is not None:
                current_alt = self.current_pose.pose.position.z
                alt_error = abs(current_alt - self.target_altitude)

                # Log current altitude
                self.get_logger().info(
                    f'Current altitude: {current_alt:.3f}m | Target: {self.target_altitude:.3f}m | Error: {alt_error:.3f}m',
                    throttle_duration_sec=1.0
                )

                # Check if we've reached target altitude
                if alt_error < self.altitude_tolerance:
                    self.get_logger().info(
                        f'Target altitude reached! Current: {current_alt:.3f}m'
                    )
                    self.get_logger().info('Startup sequence complete!')
                    self.get_logger().info('Position lock will engage automatically when ArUco tag is detected')
                    self.get_logger().info('(Position locking is handled by flight_controller_node.py)')
                    self.startup_complete = True

                    # Stop the setpoint timer
                    if hasattr(self, 'setpoint_timer'):
                        self.setpoint_timer.cancel()

                    # Keep node alive for a few more seconds to show completion message
                    self.create_timer(3.0, self.shutdown_node)

    def shutdown_node(self):
        """Shutdown the node"""
        self.get_logger().info('Auto startup node shutting down...')
        raise SystemExit


def main(args=None):
    rclpy.init(args=args)
    node = AutoStartupNode()

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
