#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from mavros_msgs.msg import State, OverrideRCIn, VfrHud
from mavros_msgs.srv import CommandBool, SetMode
from geometry_msgs.msg import PoseStamped
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
        self.aruco_detection_timeout = 30.0  # seconds to wait for ArUco
        self.aruco_stable_time = 2.0      # seconds of stable detection required

        # =====================
        # STATE
        # =====================
        self.state = None
        self.altitude = None
        self.start_altitude = None
        self.local_position = None
        self.last_position_time = None
        self.aruco_first_detected = None
        self.guided_mode_sent = False

        self.phase = 0
        self.phase_start = time.time()

        self.mode_sent = False
        self.arm_sent = False

        # Logging flags for one-time messages
        self.waiting_state_logged = False
        self.waiting_fcu_logged = False

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
            VfrHud,
            '/mavros/vfr_hud',
            self.vfr_cb,
            self.mavros_sensor_qos
        )

        self.create_subscription(
            PoseStamped,
            '/drone/local_position',
            self.position_cb,
            10
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

    def position_cb(self, msg):
        """Callback for ArUco-based local position"""
        self.local_position = msg
        self.last_position_time = time.time()

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

    def is_aruco_detected(self):
        """Check if ArUco position is being received"""
        if self.last_position_time is None:
            return False
        # Consider ArUco detected if we received position within last 0.5 seconds
        return (time.time() - self.last_position_time) < 0.5

    # --------------------------------------------------
    # MAIN LOOP
    # --------------------------------------------------

    def loop(self):

        # ---------------------
        # WAIT FOR FCU
        # ---------------------
        if self.state is None:
            if not self.waiting_state_logged:
                self.get_logger().info('Waiting for MAVROS state...')
                self.waiting_state_logged = True
            return

        if not self.state.connected:
            if not self.waiting_fcu_logged:
                self.get_logger().info('Waiting for FCU connection...')
                self.waiting_fcu_logged = True
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
        # PHASE 3 — HOVER & WAIT FOR ARUCO
        # ---------------------
        elif self.phase == 3:
            self.publish_throttle(self.hover_throttle)

            # Check if ArUco is detected
            if self.is_aruco_detected():
                if self.aruco_first_detected is None:
                    self.aruco_first_detected = time.time()
                    self.get_logger().info('ArUco marker detected!')

                # Check if detection is stable
                stable_duration = time.time() - self.aruco_first_detected
                if stable_duration >= self.aruco_stable_time:
                    self.get_logger().info(
                        f'ArUco stable for {stable_duration:.1f}s - switching to GUIDED mode'
                    )
                    self.phase = 4
                    self.phase_start = time.time()
                else:
                    self.get_logger().info(
                        f'ArUco detection stable: {stable_duration:.1f}s / {self.aruco_stable_time}s',
                        throttle_duration_sec=1.0
                    )
            else:
                # ArUco not detected - reset timer
                if self.aruco_first_detected is not None:
                    self.get_logger().warn('ArUco detection lost - waiting...')
                    self.aruco_first_detected = None

                # Check for timeout
                hover_duration = time.time() - self.phase_start
                if hover_duration > self.aruco_detection_timeout:
                    self.get_logger().error(
                        f'ArUco detection timeout after {hover_duration:.1f}s! '
                        'Manual intervention required.'
                    )
                    # Stay in hover mode for manual takeover
                else:
                    self.get_logger().info(
                        f'Waiting for ArUco detection... ({hover_duration:.0f}s / {self.aruco_detection_timeout:.0f}s)',
                        throttle_duration_sec=2.0
                    )

        # ---------------------
        # PHASE 4 — SWITCH TO GUIDED
        # ---------------------
        elif self.phase == 4:
            # Continue hovering while switching modes
            self.publish_throttle(self.hover_throttle)

            if not self.guided_mode_sent:
                self.get_logger().info('Switching to GUIDED mode for vision-based control')
                self.set_mode('GUIDED')
                self.guided_mode_sent = True

            # Wait for mode change confirmation
            if self.state.mode == 'GUIDED':
                self.get_logger().info('GUIDED mode active - vision control enabled!')
                self.get_logger().info('Auto-startup complete. Shutting down node.')
                self.phase = 5

        # ---------------------
        # PHASE 5 — COMPLETE
        # ---------------------
        elif self.phase == 5:
            # Stop publishing RC overrides - let flight controller take over
            pass


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
