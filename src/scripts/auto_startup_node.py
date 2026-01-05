#!/usr/bin/env python3
"""
Force Throttle Test - Commands upward velocity to trigger motor spin
"""

import rclpy
from rclpy.node import Node
from mavros_msgs.msg import State, PositionTarget
from mavros_msgs.srv import CommandBool, SetMode
import time


class ForceThrottleTestNode(Node):
    def __init__(self):
        super().__init__('force_throttle_test_node')
        
        self.mavros_state = None
        
        # Subscribers
        self.state_sub = self.create_subscription(
            State,
            '/mavros/state',
            self.state_callback,
            10
        )
        
        # Publishers - Use PositionTarget for velocity control
        self.setpoint_raw_pub = self.create_publisher(
            PositionTarget,
            '/mavros/setpoint_raw/local',
            10
        )
        
        # Service clients
        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')
        
        # Timer for continuous publishing
        self.setpoint_timer = self.create_timer(0.05, self.publish_velocity_setpoint)
        self.target_velocity = 0.0
        
        # Wait for services
        self.arming_client.wait_for_service()
        self.set_mode_client.wait_for_service()
        
    def state_callback(self, msg):
        self.mavros_state = msg
    
    def publish_velocity_setpoint(self):
        msg = PositionTarget()

        msg.header.stamp = self.get_clock().now().to_msg()

        # ArduPilot expects LOCAL_NED for guided velocity
        msg.coordinate_frame = PositionTarget.FRAME_LOCAL_NED

        # Use ONLY velocity control
        msg.type_mask = (
            PositionTarget.IGNORE_PX |
            PositionTarget.IGNORE_PY |
            PositionTarget.IGNORE_PZ |
            PositionTarget.IGNORE_AFX |
            PositionTarget.IGNORE_AFY |
            PositionTarget.IGNORE_AFZ |
            PositionTarget.IGNORE_YAW |
            PositionTarget.IGNORE_YAW_RATE
        )

        # Velocity command (NED frame)
        msg.velocity.x = 0.0
        msg.velocity.y = 0.0

        # NED: negative Z = UP
        msg.velocity.z = -self.target_velocity

        self.setpoint_raw_pub.publish(msg)

    
    def wait_for_state(self, timeout=10.0):
        start_time = time.time()
        while self.mavros_state is None:
            rclpy.spin_once(self, timeout_sec=0.1)
            if time.time() - start_time > timeout:
                return False
        return True
    
    def set_mode(self, mode):
        req = SetMode.Request()
        req.custom_mode = mode
        future = self.set_mode_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        return future.result().mode_sent if future.result() else False
    
    def arm(self):
        req = CommandBool.Request()
        req.value = True
        future = self.arming_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        return future.result().success if future.result() else False
    
    def force_throttle_test(self):
        """Test with velocity command to force throttle"""
        self.get_logger().info('=' * 60)
        self.get_logger().info('FORCE THROTTLE TEST')
        self.get_logger().info('=' * 60)
        
        if not self.wait_for_state():
            self.get_logger().error('❌ Failed to connect to MAVROS')
            return False
        
        # Set GUIDED mode
        self.get_logger().info('Setting GUIDED mode...')
        if not self.set_mode('GUIDED'):
            return False
        time.sleep(1.0)
        
        # Start velocity setpoint
        self.get_logger().info('Starting velocity setpoint (0 m/s)...')
        self.target_velocity = 0.0
        time.sleep(2.0)
        
        # Arm
        self.get_logger().info('Arming...')
        if not self.arm():
            return False
        time.sleep(2.0)
        
        # Command upward velocity
        self.get_logger().info('')
        self.get_logger().info('⚠️  COMMANDING UPWARD VELOCITY - MOTORS SHOULD SPIN!')
        self.get_logger().info('Ramping up velocity command...')
        
        for vel in [0.2, 0.5, 0.8, 1.0, 0.8, 0.5, 0.2, 0.0]:
            self.target_velocity = vel
            self.get_logger().info(f'Target velocity: {vel} m/s upward')
            
            # Monitor servo output
            self.get_logger().info('Check servo outputs now with:')
            self.get_logger().info('  ros2 topic echo /mavros/servo_output_raw')
            time.sleep(3.0)
        
        self.get_logger().info('')
        self.get_logger().info('✓ Test complete')
        return True


def main(args=None):
    rclpy.init(args=args)
    node = ForceThrottleTestNode()
    
    try:
        success = node.force_throttle_test()
        if success:
            node.get_logger().info('Monitor servo outputs - values should increase above 1100')
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()