#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from mavros_msgs.msg import State, OverrideRCIn, VFRHUD
from mavros_msgs.srv import CommandBool, SetMode
import time


class AltHoldAutonomousTakeoff(Node):
    def __init__(self):
        super().__init__('alt_hold_autonomous_takeoff')

        # =====================
        # PARAMETERS
        # =====================
        self.target_altitude = 1.0        # meters
        self.spinup_throttle = 1150       # motor idle (baro unlock)
        self.takeoff_throttle = 1650      # climb
        self.hover_throttle = 1500        # neutral
        self.spinup_time = 1.5            # seconds
        self.max_takeoff_time = 6.0       # seconds failsafe

        # =====================
        # STATE
        # =====================
        self.state = None
        self.altitude = None
        self.start_altitude = None

        self.phase = 0
        self.phase_start = time.time()

        self.mode_sent = False
        self.arm_sent = False

        # =====================
        # QoS
        # =====================
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
            VFRHUD,
            '/mavros/vfr_hud',
            self.vfr_cb,
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

    def vfr_cb(self, msg):
        self.altitude = msg.altitude

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
        msg.channels[2] = pwm  # throttle = channel 3
        self.rc_pub.publish(msg)

    # --------------------------------------------------
    # MAIN LOOP
    # --------------------------------------------------

    def loop(self):

        # ---------------------
        # WAIT FOR FCU
        # ---------------------
        if self.state is None:
            self.get_logger().info_once('Waiting for MAVROS state...')
            return

        if not self.state.connected:
            self.get_logger().info_once('Waiting for FCU connection...')
            return

        # ---------------------
        # PHASE 0 — MODE + ARM
        # ---------------------
        if self.phase == 0:

            if not self.mode_sent:
                self.get_logger().info('Setting ALT_HOLD mode')
                self.set_mode('ALT_HOLD')
                self.mode_sent = True
                return

            if not self.arm_sent:
                self.get_logger().info('Arming vehicle')
                self.arm()
                self.arm_sent = True
                return

            if self.state.armed:
                self.get_logger().info('Vehicle armed')
                self.phase = 1
                self.phase_start = time.time()

        # ---------------------
        # PHASE 1 — MOTOR SPINUP
        # ---------------------
        elif self.phase == 1:
            self.publish_throttle(self.spinup_throttle)

            if time.time() - self.phase_start > self.spinup_time:
                self.get_logger().info('Motors spinning, starting climb')
                self.phase = 2
                self.phase_start = time.time()

        # ---------------------
        # PHASE 2 — CLIMB
        # ---------------------
        elif self.phase == 2:

            # capture start altitude AFTER motors spin
            if self.start_altitude is None and self.altitude is not None:
                self.start_altitude = self.altitude
                self.get_logger().info(
                    f'Start altitude captured: {self.start_altitude:.2f} m'
                )

            if self.start_altitude is None:
                return

            current_height = self.altitude - self.start_altitude

            if current_height < self.target_altitude:
                self.publish_throttle(self.takeoff_throttle)
            else:
                self.get_logger().info(
                    f'Target altitude reached: {current_height:.2f} m'
                )
                self.phase = 3

            if time.time() - self.phase_start > self.max_takeoff_time:
                self.get_logger().error('Takeoff timeout!')
                self.phase = 3

        # ---------------------
        # PHASE 3 — HOVER
        # ---------------------
        elif self.phase == 3:
            self.publish_throttle(self.hover_throttle)
            # wait for GUIDED + vision takeover


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
