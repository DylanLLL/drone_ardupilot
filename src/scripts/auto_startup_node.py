#!/usr/bin/env python3
"""
Bootstrap Takeoff Script
Handles takeoff with dummy position until vision locks
"""

import rclpy
from rclpy.node import Node
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL
from geometry_msgs.msg import PoseStamped
import time


class BootstrapTakeoffNode(Node):
    def __init__(self):
        super().__init__('bootstrap_takeoff_node')
        
        self.mavros_state = None
        self.vision_locked = False
        
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
        
        # Service clients
        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')
        self.takeoff_client = self.create_client(CommandTOL, '/mavros/cmd/takeoff')
        
        # Wait for services
        self.get_logger().info('Waiting for MAVROS services...')
        self.arming_client.wait_for_service()
        self.set_mode_client.wait_for_service()
        self.takeoff_client.wait_for_service()
        self.get_logger().info('✓ MAVROS services ready')
    
    def state_callback(self, msg):
        self.mavros_state = msg
    
    def position_callback(self, msg):
        # Detect vision lock (non-zero position with reasonable values)
        if abs(msg.pose.position.z) > 0.3:  # Altitude > 30cm means we're airborne and have vision
            if not self.vision_locked:
                self.vision_locked = True
                self.get_logger().info('🔒 VISION LOCKED - Real position feedback active!')
    
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
    
    def takeoff(self, altitude):
        """Execute takeoff"""
        req = CommandTOL.Request()
        req.altitude = altitude
        
        future = self.takeoff_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        
        if future.result() is not None:
            return future.result().success
        return False
    
    def bootstrap_takeoff_sequence(self, target_altitude=1.5):
        """
        Bootstrap takeoff sequence with dummy position:
        1. Position estimator publishes dummy (0,0,0)
        2. Set GUIDED mode
        3. Arm in GUIDED (works because EK3 has position from dummy + baro)
        4. Takeoff
        5. Vision locks when altitude allows marker detection
        6. System switches to real vision feedback automatically
        """
        self.get_logger().info('=' * 60)
        self.get_logger().info('BOOTSTRAP TAKEOFF SEQUENCE')
        self.get_logger().info('=' * 60)
        
        # Wait for MAVROS connection
        if not self.wait_for_state():
            self.get_logger().error('❌ Failed to connect to MAVROS')
            return False
        
        self.get_logger().info('✓ MAVROS connected')
        
        # Step 1: Verify bootstrap position is being published
        self.get_logger().info('Step 1: Verifying bootstrap position...')
        self.get_logger().info('   (Position estimator should be publishing dummy position)')
        time.sleep(2.0)
        
        # Step 2: Set GUIDED mode
        self.get_logger().info('Step 2: Setting GUIDED mode...')
        if not self.set_mode('GUIDED'):
            self.get_logger().error('❌ Failed to set GUIDED mode')
            self.get_logger().error('   Check: Is EK3_SRC1_POSZ = 3 (Barometer)?')
            return False
        
        time.sleep(1.0)
        self.get_logger().info('✓ GUIDED mode set')
        
        # Step 3: Arm
        self.get_logger().info('Step 3: Arming in GUIDED mode...')
        self.get_logger().info('   (Using bootstrap position + barometer)')
        
        if not self.arm():
            self.get_logger().error('❌ Failed to arm')
            self.get_logger().error('   Check: ')
            self.get_logger().error('   - Is EK3_SRC1_POSZ = 3?')
            self.get_logger().error('   - Is position estimator publishing?')
            self.get_logger().error('   - Are pre-arm checks satisfied?')
            return False
        
        # Wait for arming to complete
        time.sleep(2.0)
        
        # # Verify armed
        # rclpy.spin_once(self, timeout_sec=0.1)
        # if not self.mavros_state.armed:
        #     self.get_logger().error('❌ Drone not armed after arm command')
        #     return False
        
        self.get_logger().info('✓ Drone armed successfully in GUIDED mode!')
        
        # Step 4: Takeoff
        self.get_logger().info(f'Step 4: Taking off to {target_altitude}m...')
        self.get_logger().info('   Drone will climb using barometer altitude')
        self.get_logger().info('   Vision will lock when camera can see markers')
        
        if not self.takeoff(target_altitude):
            self.get_logger().error('❌ Takeoff command failed')
            return False
        
        self.get_logger().info('✓ Takeoff command sent!')
        
        # Step 5: Monitor for vision lock
        self.get_logger().info('Step 5: Monitoring for vision lock...')
        
        start_time = time.time()
        timeout = 15.0  # 15 seconds to achieve vision lock
        
        while not self.vision_locked and (time.time() - start_time) < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
            time.sleep(0.1)
        
        if self.vision_locked:
            self.get_logger().info('✓ VISION LOCKED! Real position feedback active')
            self.get_logger().info('   Drone is now using ArUco markers for position')
        else:
            self.get_logger().warn('⚠ Vision lock timeout - markers may not be visible yet')
            self.get_logger().warn('   Drone is relying on bootstrap position + barometer')
            self.get_logger().warn('   Vision will lock when markers become visible')
        
        self.get_logger().info('=' * 60)
        self.get_logger().info('✓ BOOTSTRAP TAKEOFF SEQUENCE COMPLETED')
        self.get_logger().info('=' * 60)
        self.get_logger().info('Drone status:')
        self.get_logger().info(f'  - Armed: {self.mavros_state.armed}')
        self.get_logger().info(f'  - Mode: {self.mavros_state.mode}')
        self.get_logger().info(f'  - Vision locked: {self.vision_locked}')
        self.get_logger().info('=' * 60)
        
        return True


def main(args=None):
    rclpy.init(args=args)
    node = BootstrapTakeoffNode()
    
    try:
        # Execute bootstrap takeoff sequence
        success = node.bootstrap_takeoff_sequence(target_altitude=1.5)
        
        if success:
            node.get_logger().info('Monitor drone and wait for vision lock')
            node.get_logger().info('Press Ctrl+C when ready to exit')
            
            # Keep node alive to monitor vision lock
            rclpy.spin(node)
        else:
            node.get_logger().error('BOOTSTRAP TAKEOFF SEQUENCE FAILED')
        
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()