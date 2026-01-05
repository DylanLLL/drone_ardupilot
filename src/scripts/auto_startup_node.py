#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from mavros_msgs.msg import State, PositionTarget, AttitudeTarget
from mavros_msgs.srv import CommandBool, SetMode
import time
import math


class AutonomousTakeoffNode(Node):
    def __init__(self):
        super().__init__('autonomous_takeoff_node')

        self.state = None

        # Subscribers
        self.create_subscription(
            State,
            '/mavros/state',
            self.state_cb,
            10
        )

        # Publishers
        self.att_pub = self.create_publisher(
            AttitudeTarget,
            '/mavros/setpoint_raw/attitude',
            10
        )

        self.vel_pub = self.create_publisher(
            PositionTarget,
            '/mavros/setpoint_raw/local',
            10
        )

        # Services
        self.arm_srv = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.mode_srv = self.create_client(SetMode, '/mavros/set_mode')

        self.arm_srv.wait_for_service()
        self.mode_srv.wait_for_service()

        # Timer (20 Hz)
        self.timer = self.create_timer(0.05, self.loop)

        self.phase = 0
        self.phase_start = time.time()

    def state_cb(self, msg):
        self.state = msg

    def set_mode(self, mode):
        req = SetMode.Request()
        req.custom_mode = mode
        self.mode_srv.call_async(req)

    def arm(self):
        req = CommandBool.Request()
        req.value = True
        self.arm_srv.call_async(req)

    def publish_bootstrap_thrust(self, thrust):
        msg = AttitudeTarget()
        msg.header.stamp = self.get_clock().now().to_msg()

        msg.type_mask = (
            AttitudeTarget.IGNORE_ROLL_RATE |
            AttitudeTarget.IGNORE_PITCH_RATE |
            AttitudeTarget.IGNORE_YAW_RATE |
            AttitudeTarget.IGNORE_ATTITUDE
        )

        msg.thrust = thrust
        self.att_pub.publish(msg)

    def publish_velocity(self, vz):
        msg = PositionTarget()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.coordinate_frame = PositionTarget.FRAME_LOCAL_NED

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

        msg.velocity.x = 0.0
        msg.velocity.y = 0.0
        msg.velocity.z = -vz  # NED: negative = up

        self.vel_pub.publish(msg)

    def loop(self):
        if self.state is None:
            return

        # Phase 0 — Setup
        if self.phase == 0:
            self.get_logger().info('Setting GUIDED and arming')
            self.set_mode('GUIDED')
            self.arm()
            self.phase = 1
            self.phase_start = time.time()

        # Phase 1 — Bootstrap thrust
        elif self.phase == 1:
            self.publish_bootstrap_thrust(thrust=0.18)

            if time.time() - self.phase_start > 1.0:
                self.get_logger().info('Bootstrap complete, switching to velocity climb')
                self.phase = 2
                self.phase_start = time.time()

        # Phase 2 — Velocity climb
        elif self.phase == 2:
            self.publish_velocity(vz=0.4)

            if time.time() - self.phase_start > 3.0:
                self.get_logger().info('Takeoff complete')
                self.phase = 3

        # Phase 3 — Hover (zero climb)
        elif self.phase == 3:
            self.publish_velocity(vz=0.0)


def main():
    rclpy.init()
    node = AutonomousTakeoffNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
