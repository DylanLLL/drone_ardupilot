#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from mavros_msgs.msg import State, OverrideRCIn, Altitude
from mavros_msgs.srv import CommandBool, SetMode
import time


class AltHoldAutonomousTakeoff(Node):
    def __init__(self):
        super().__init__('alt_hold_autonomous_takeoff')

        # =====================
        # PARAMETERS
        # =====================
        self.target_altitude = 1.0      # meters
        self.takeoff_throttle = 1650    # climb
        self.hover_throttle = 1500      # neutral
        self.max_takeoff_time = 5.0     # seconds (failsafe)

        # =====================
        # STATE
        # =====================
        self.state = None
        self.altitude = None
        self.start_altitude = None
        self.phase = 0
        self.phase_start = time.time()

        self.mavros_sensor_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT
        )

        self.mavros_cmd_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE
        )

        # =====================
        # SUBSCRIBERS
        # =====================
        self.create_subscription(
            State,
            '/mavros/state',
            self.state_cb,
            self.mavros_sensor_qos
        )

        self.create_subscription(
            Altitude,
            '/mavros/altitude',
            self.altitude_cb,
            self.mavros_sensor_qos
        )

        # =====================
        # PUBLISHERS
        # =====================
        self.rc_pub = self.create_publisher(
            OverrideRCIn,
            '/mavros/rc/override',
            self.mavros_cmd_qos
        )

        # =====================
        # SERVICES
        # =====================
        self.arm_srv = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.mode_srv = self.create_client(SetMode, '/mavros/set_mode')

        self.arm_srv.wait_for_service()
        self.mode_srv.wait_for_service()

        # =====================
        # TIMER
        # =====================
        self.timer = self.create_timer(0.05, self.loop)  # 20 Hz

    # --------------------------------------------------
    # CALLBACKS
    # --------------------------------------------------

    def state_cb(self, msg):
        self.state = msg

    def altitude_cb(self, msg):
        self.altitude = msg.relative

    # --------------------------------------------------
    # HELPERS
    # --------------------------------------------------

    def set_mode(self, mode):
        req = SetMode.Request()
        req.custom_mode = mode
        self.mode_srv.call_async(req)

    def arm(self):
        req = CommandBool.Request()
        req.value = True
        self.arm_srv.call_async(req)

    def publish_throttle(self, pwm):
        msg = OverrideRCIn()
        msg.channels = [0] * 18
        msg.channels[2] = pwm  # RC channel 3 = throttle
        self.rc_pub.publish(msg)

    # --------------------------------------------------
    # MAIN LOOP
    # --------------------------------------------------

    def loop(self):
        if self.state is None or self.altitude is None:
            return

        # Phase 0 — setup
        if self.phase == 0:
            self.get_logger().info('Setting ALT_HOLD and arming')
            self.set_mode('ALT_HOLD')
            self.arm()
            self.phase = 1
            self.phase_start = time.time()

        # Phase 1 — record ground altitude
        elif self.phase == 1:
            self.start_altitude = self.altitude
            self.get_logger().info(
                f'Ground altitude recorded: {self.start_altitude:.2f} m'
            )
            self.phase = 2
            self.phase_start = time.time()

        # Phase 2 — climb
        elif self.phase == 2:
            current_height = self.altitude - self.start_altitude

            if current_height < self.target_altitude:
                self.publish_throttle(self.takeoff_throttle)
            else:
                self.get_logger().info(
                    f'Target altitude reached: {current_height:.2f} m'
                )
                self.phase = 3

            # failsafe
            if time.time() - self.phase_start > self.max_takeoff_time:
                self.get_logger().error('Takeoff timeout!')
                self.phase = 3

        # Phase 3 — hold altitude
        elif self.phase == 3:
            self.publish_throttle(self.hover_throttle)
            # stay here until external node switches to GUIDED

# --------------------------------------------------
# MAIN
# --------------------------------------------------

def main():
    rclpy.init()
    node = AltHoldAutonomousTakeoff()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
