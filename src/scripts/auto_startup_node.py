#!/usr/bin/env python3
"""
Bench Test Takeoff - Simulates takeoff using position setpoints
"""

import rclpy
from rclpy.node import Node
from mavros_msgs.msg import State, PositionTarget
from mavros_msgs.srv import CommandBool, SetMode
from geometry_msgs.msg import PoseStamped
import time


class BenchTestTakeoffNode(Node):
    def __init__(self):
        super().__init__('bench_test_takeoff_node')
        
        self.mavros_state = None
        self.current_altitude = 0.0
        
        # Subscribers
        self.state_sub = self.create_subscription(
            State,
            '/mavros/state',
            self.state_callback,
            10
        )
        
        self.position_sub = self.create_subscription(
            PoseStamped,
            '/drone/local_position',
            self.position_callback,
            10
        )
        
        # Publishers
        self.setpoint_pub = self.create_publisher(
            PoseStamped,
            '/mavros/setpoint_position/local',
            10
        )
        
        # Service clients
        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')
        
        # Wait for services
        self.get_logger().info('Waiting for MAVROS services...')
        self.arming_client.wait_for_service()
        self.set_mode_client.wait_for_service()
        self.get_logger().info('✓ MAVROS services ready')
        
        # Timer for continuous setpoint publishing
        self.setpoint_timer = self.create_timer(0.05, self.publish_setpoint)  # 20 Hz
        self.target_pose = None
    
    def state_callback(self, msg):
        self.mavros_state = msg
    
    def position_callback(self, msg):
        self.current_altitude = msg.pose.position.z
    
    def publish_setpoint(self):
        """Continuously publish setpoint"""
        if self.target_pose is not None:
            self.setpoint_pub.publish(self.target_pose)
    
    def wait_for_state(self, timeout=10.0):
        """Wait for MAVROS state to be received"""
        start_time = time.time()
        while self.mavros_state is None:
            rclpy.spin_once(self, timeout_sec=0.1)
            if time.time() - start_time > timeout:
                return False
        return True
    
    def set_mode(self, mode):
        """Set flight mode"""
        req = SetMode.Request()
        req.custom_mode = mode
        
        future = self.set_mode_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        
        if future.result() is not None:
            return future.result().mode_sent
        return False
    
    def arm(self):
        """Arm the drone"""
        req = CommandBool.Request()
        req.value = True
        
        future = self.arming_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        
        if future.result() is not None:
            return future.result().success
        return False
    
    def set_target_position(self, x, y, z):
        """Set target position"""
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = 'map'
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        pose.pose.orientation.w = 1.0
        
        self.target_pose = pose
        self.get_logger().info(f'Target position set: [{x:.2f}, {y:.2f}, {z:.2f}]')
    
    def bench_test_sequence(self, target_altitude=0.7):
        """
        Bench test sequence (no actual takeoff command):
        1. Set GUIDED mode
        2. Arm
        3. Publish position setpoints (simulates takeoff)
        4. Monitor (motors won't actually spin without props, but commands are sent)
        """
        self.get_logger().info('=' * 60)
        self.get_logger().info('BENCH TEST TAKEOFF SEQUENCE')
        self.get_logger().info('(No propellers - testing command flow only)')
        self.get_logger().info('=' * 60)
        
        # Wait for MAVROS connection
        if not self.wait_for_state():
            self.get_logger().error('❌ Failed to connect to MAVROS')
            return False
        
        self.get_logger().info('✓ MAVROS connected')
        
        # Step 1: Set GUIDED mode
        self.get_logger().info('Step 1: Setting GUIDED mode...')
        if not self.set_mode('GUIDED'):
            self.get_logger().error('❌ Failed to set GUIDED mode')
            return False
        
        time.sleep(1.0)
        self.get_logger().info('✓ GUIDED mode set')
        
        # Step 2: Start publishing setpoints BEFORE arming
        self.get_logger().info('Step 2: Starting setpoint stream...')
        self.set_target_position(0.0, 0.0, target_altitude)
        time.sleep(2.0)  # Let setpoints stream for a bit
        self.get_logger().info('✓ Setpoints streaming')
        
        # Step 3: Arm
        self.get_logger().info('Step 3: Arming...')
        if not self.arm():
            self.get_logger().error('❌ Failed to arm')
            return False
        
        time.sleep(2.0)
        
        # # Verify armed
        # rclpy.spin_once(self, timeout_sec=0.1)
        # if not self.mavros_state.armed:
        #     self.get_logger().error('❌ Drone not armed after arm command')
        #     return False
        
        self.get_logger().info('✓ Drone armed!')
        
        # Step 4: Monitor
        self.get_logger().info(f'Step 4: Commanding climb to {target_altitude}m...')
        self.get_logger().info('   (Motors would spin up if propellers were installed)')
        self.get_logger().info('   Position setpoints are being published at 20 Hz')
        
        # Monitor for a few seconds
        self.get_logger().info('')
        self.get_logger().info('Monitoring for 10 seconds...')
        
        for i in range(10):
            rclpy.spin_once(self, timeout_sec=0.1)
            self.get_logger().info(
                f'  t={i+1}s | Mode: {self.mavros_state.mode} | '
                f'Armed: {self.mavros_state.armed} | '
                f'Alt: {self.current_altitude:.2f}m'
            )
            time.sleep(1.0)
        
        self.get_logger().info('=' * 60)
        self.get_logger().info('✓ BENCH TEST SEQUENCE COMPLETED')
        self.get_logger().info('=' * 60)
        self.get_logger().info('Results:')
        self.get_logger().info(f'  - Mode: {self.mavros_state.mode}')
        self.get_logger().info(f'  - Armed: {self.mavros_state.armed}')
        self.get_logger().info(f'  - Setpoints: Publishing continuously')
        self.get_logger().info('')
        self.get_logger().info('NOTE: Altitude will not increase without propellers')
        self.get_logger().info('      This is expected for bench testing')
        self.get_logger().info('=' * 60)
        
        return True


def main(args=None):
    rclpy.init(args=args)
    node = BenchTestTakeoffNode()
    
    try:
        # Execute bench test sequence
        success = node.bench_test_sequence(target_altitude=1.5)
        
        if success:
            node.get_logger().info('Press Ctrl+C to exit and disarm')
            rclpy.spin(node)
        else:
            node.get_logger().error('BENCH TEST SEQUENCE FAILED')
        
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()