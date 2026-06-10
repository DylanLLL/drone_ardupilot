#!/usr/bin/env python3
"""
Flight Controller Node
Handles high-level flight commands and interfaces with ArduPilot via MAVROS
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseArray
from geographic_msgs.msg import GeoPointStamped
from mavros_msgs.msg import State, OverrideRCIn, PositionTarget
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL
from std_msgs.msg import String
import numpy as np
from typing import Optional


class FlightControllerNode(Node):
    """
    High-level flight controller for autonomous navigation
    """

    def __init__(self):
        super().__init__('flight_controller')

        # Declare parameters
        self.declare_parameter('default_altitude', 1.5)
        self.declare_parameter('takeoff_altitude', 1.5)
        self.declare_parameter('max_velocity', 1.0)
        self.declare_parameter('position_tolerance', 0.15)
        self.declare_parameter('position_deadband', 0.05)
        self.declare_parameter('control_rate', 20.0)

        # PID gains for XY position hold (tune these on the actual drone)
        # Start conservative: low Kp prevents oscillation, small Ki corrects steady-state drift
        self.declare_parameter('pid_kp', 0.4)    # proportional: immediate response to error
        self.declare_parameter('pid_ki', 0.05)   # integral: corrects persistent drift
        self.declare_parameter('pid_kd', 0.1)    # derivative: damps oscillations
        self.declare_parameter('pid_max_vel', 0.5)           # m/s: velocity cap per axis
        self.declare_parameter('pid_integral_max', 0.5)      # m·s: anti-windup clamp

        # Vision-loss failsafe: command LAND if the marker stays lost this long while holding
        self.declare_parameter('vision_loss_timeout', 2.0)   # seconds

        # GPS-denied arming: set the EKF origin so position-controlled modes can arm
        # without a GPS module. Required when GPS_TYPE=0 and the EKF position source is
        # ExternalNav (vision). The absolute lat/lon is arbitrary for local vision nav.
        self.declare_parameter('set_ekf_origin', True)
        self.declare_parameter('ekf_origin_lat', 0.0)
        self.declare_parameter('ekf_origin_lon', 0.0)
        self.declare_parameter('ekf_origin_alt', 0.0)

        # Get parameters
        self.default_altitude = self.get_parameter('default_altitude').value
        self.takeoff_altitude = self.get_parameter('takeoff_altitude').value
        self.max_velocity = self.get_parameter('max_velocity').value
        self.position_tolerance = self.get_parameter('position_tolerance').value
        self.position_deadband = self.get_parameter('position_deadband').value
        control_rate = self.get_parameter('control_rate').value

        self.pid_kp = self.get_parameter('pid_kp').value
        self.pid_ki = self.get_parameter('pid_ki').value
        self.pid_kd = self.get_parameter('pid_kd').value
        self.pid_max_vel = self.get_parameter('pid_max_vel').value
        self.pid_integral_max = self.get_parameter('pid_integral_max').value
        self.vision_loss_timeout = self.get_parameter('vision_loss_timeout').value
        self.set_ekf_origin = self.get_parameter('set_ekf_origin').value
        self.ekf_origin_lat = self.get_parameter('ekf_origin_lat').value
        self.ekf_origin_lon = self.get_parameter('ekf_origin_lon').value
        self.ekf_origin_alt = self.get_parameter('ekf_origin_alt').value

        # State variables
        self.mavros_state: Optional[State] = None
        self.current_pose: Optional[PoseStamped] = None
        self.target_pose: Optional[PoseStamped] = None
        self.mavros_connected_logged = False
        self.last_mode = ""  # Track mode changes for logging
        self.latest_aruco_detection: Optional[PoseArray] = None  # Latest ArUco detections
        self.position_locked = False  # Track if we've locked position in GUIDED mode
        self.last_aruco_time = None          # timestamp of most recent ArUco detection
        self.vision_loss_landing = False     # one-shot guard so LAND is commanded only once
        self.ekf_origin_sent_count = 0       # how many times the EKF origin has been pushed

        # PID controller state (XY only; Z/altitude is handled by position setpoint)
        self.pid_integral = np.zeros(2)       # accumulated error × time
        self.pid_prev_error = np.zeros(2)     # previous cycle error for derivative term
        self.pid_last_time = None             # timestamp of previous control cycle

        # PositionTarget type_mask: velocity XY + position Z; keep current heading.
        # Computed once — bits are fixed for the lifetime of the node.
        self._setpoint_type_mask = (
            PositionTarget.IGNORE_PX | PositionTarget.IGNORE_PY |
            PositionTarget.IGNORE_VZ |
            PositionTarget.IGNORE_AFX | PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ |
            PositionTarget.IGNORE_YAW |
            PositionTarget.IGNORE_YAW_RATE
        )
        
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

        self.aruco_sub = self.create_subscription(
            PoseArray,
            '/aruco/poses',
            self.aruco_callback,
            10
        )

        # Publishers
        self.setpoint_position_pub = self.create_publisher(
            PoseStamped,
            '/mavros/setpoint_position/local',
            10
        )

        # Velocity+position setpoint for PID-driven control (XY velocity + Z altitude)
        self.setpoint_raw_pub = self.create_publisher(
            PositionTarget,
            '/mavros/setpoint_raw/local',
            10
        )

        # RC Override publisher (for disabling RC in GUIDED mode)
        self.rc_override_pub = self.create_publisher(
            OverrideRCIn,
            '/mavros/rc/override',
            10
        )

        # EKF origin publisher — lets the FCU arm in GPS-denied mode (no GPS module)
        self.set_gp_origin_pub = self.create_publisher(
            GeoPointStamped,
            '/mavros/global_position/set_gp_origin',
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

        # MAVROS connection check timer (non-blocking)
        self.mavros_check_timer = self.create_timer(1.0, self._check_mavros_connection)

        self.get_logger().info('Flight Controller Node initialized')
        self.get_logger().info(
            f'XY PID: Kp={self.pid_kp:.3f} Ki={self.pid_ki:.3f} '
            f'Kd={self.pid_kd:.3f} max_vel={self.pid_max_vel:.3f}m/s '
            f'deadband={self.position_deadband:.3f}m'
        )
    
    def _check_mavros_connection(self):
        """Non-blocking check for MAVROS connection"""
        if self.mavros_state is not None and self.mavros_state.connected:
            if not self.mavros_connected_logged:
                self.get_logger().info('MAVROS connected!')
                self.mavros_connected_logged = True
            # Push the EKF origin a few times once connected so the FCU can arm
            # without a GPS module. Repeated because the first send may arrive
            # before the EKF is ready to accept it.
            if self.set_ekf_origin and self.ekf_origin_sent_count < 3:
                self._publish_ekf_origin()
                self.ekf_origin_sent_count += 1
        else:
            if self.mavros_connected_logged:
                self.get_logger().warn('MAVROS disconnected!')
                self.mavros_connected_logged = False

    def _publish_ekf_origin(self):
        """
        Publish a fixed global origin to MAVROS so the EKF initializes in GPS-denied
        mode. The absolute lat/lon is arbitrary for local vision navigation — it only
        needs to exist so position-controlled modes (GUIDED) can pass arming checks.
        """
        msg = GeoPointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.position.latitude = self.ekf_origin_lat
        msg.position.longitude = self.ekf_origin_lon
        msg.position.altitude = self.ekf_origin_alt
        self.set_gp_origin_pub.publish(msg)
        self.get_logger().info(
            f'Published EKF origin ({self.ekf_origin_lat}, {self.ekf_origin_lon}, '
            f'{self.ekf_origin_alt}) for GPS-denied arming'
        )

    def _start_position_hold(self):
        """
        Lock position in place when entering GUIDED mode.
        Requires ArUco marker to be detected to have valid position estimate.
        The locked position is relative to the ArUco marker that was just seen.
        """
        # Check if we have valid position estimate (requires ArUco detection)
        if self.current_pose is None:
            self.get_logger().warn('=' * 60)
            self.get_logger().warn('Cannot lock position - no position estimate available')
            self.get_logger().warn('ArUco marker must be visible to camera for position lock')
            self.get_logger().warn('Point camera at ArUco marker and try switching to GUIDED again')
            self.get_logger().warn('=' * 60)
            return

        # Check if ArUco marker is currently detected
        if self.latest_aruco_detection is None or len(self.latest_aruco_detection.poses) == 0:
            self.get_logger().warn('=' * 60)
            self.get_logger().warn('Cannot lock position - no ArUco marker currently detected')
            self.get_logger().warn('Point camera at ArUco marker and try switching to GUIDED again')
            self.get_logger().warn('=' * 60)
            return

        # CRITICAL: Take a SNAPSHOT of the world position at this moment
        # This creates a fixed target in the world frame that won't change
        # even if the camera's view of the marker changes slightly
        locked_x = self.current_pose.pose.position.x
        locked_y = self.current_pose.pose.position.y
        locked_z = self.current_pose.pose.position.z
        locked_orientation = self.current_pose.pose.orientation

        # Get marker info for logging
        marker_camera = self.latest_aruco_detection.poses[0]
        cam_x = marker_camera.position.x
        cam_y = marker_camera.position.y
        cam_z = marker_camera.position.z

        # Create a FIXED target pose in world frame
        # This target will NOT update even as current_pose updates from new camera readings
        target = PoseStamped()
        target.header.stamp = self.get_clock().now().to_msg()
        target.header.frame_id = 'map'  # World frame
        target.pose.position.x = locked_x
        target.pose.position.y = locked_y
        target.pose.position.z = locked_z
        target.pose.orientation = locked_orientation

        self.target_pose = target
        self.position_locked = True
        # Reset PID so integral from previous target doesn't corrupt the new hold
        self.pid_integral = np.zeros(2)
        self.pid_prev_error = np.zeros(2)
        self.pid_last_time = None

        # Log the locked position
        self.get_logger().info('=' * 60)
        self.get_logger().info('GUIDED Mode: Position LOCKED in world frame')
        self.get_logger().info(f'Marker detected at camera offset: X={cam_x:.3f}m, Y={cam_y:.3f}m, Z={cam_z:.3f}m')
        self.get_logger().info(f'LOCKED world position: [{locked_x:.3f}, {locked_y:.3f}, {locked_z:.3f}]')
        self.get_logger().info(f'Frame: {target.header.frame_id}')
        self.get_logger().info('This position will remain fixed regardless of camera movement')
        self.get_logger().info('=' * 60)

    def state_callback(self, msg: State):
        """Callback for MAVROS state"""
        # Detect mode changes
        if self.mavros_state is not None:
            previous_mode = self.mavros_state.mode

            # Switching TO GUIDED mode
            if previous_mode != 'GUIDED' and msg.mode == 'GUIDED':
                # Just switched to GUIDED mode - start position hold
                self.get_logger().info('Switched to GUIDED mode - attempting position lock')
                self._start_position_hold()

            # Switching FROM GUIDED mode to something else
            elif previous_mode == 'GUIDED' and msg.mode != 'GUIDED':
                # Left GUIDED mode - reset lock and PID state
                self.position_locked = False
                self.vision_loss_landing = False
                self.pid_integral = np.zeros(2)
                self.pid_prev_error = np.zeros(2)
                self.pid_last_time = None
                self.get_logger().info(f'Left GUIDED mode (now in {msg.mode}) - position lock and PID reset')

        self.mavros_state = msg
    
    def local_position_callback(self, msg: PoseStamped):
        """Callback for drone local position"""
        self.current_pose = msg

    def aruco_callback(self, msg: PoseArray):
        """Callback for ArUco marker detections"""
        self.latest_aruco_detection = msg
        if msg.poses:
            self.last_aruco_time = self.get_clock().now()
            self.vision_loss_landing = False  # fresh detection re-arms the failsafe
    
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
        elif command == 'hold' or command == 'hold_position':
            self._start_position_hold()
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
        """
        Main control loop: XY velocity PID + Z altitude position setpoint.

        Instead of publishing a raw position target (which relies on ArduPilot's
        GPS-tuned position P loop), we compute velocity commands with our own PID
        tuned for the vision-based position estimates. Z altitude is still handled
        as a position setpoint since the barometer/ALT_HOLD is more reliable there.

        Tuning guide (conservative starting values):
          pid_kp: increase if drone responds too slowly, decrease if oscillating
          pid_ki: increase if drone settles with a steady-state offset (drift)
          pid_kd: increase if drone overshoots and oscillates after disturbance
        """
        self._manage_rc_override()

        is_armed = self.mavros_state is not None and self.mavros_state.armed
        is_guided = self.mavros_state is not None and self.mavros_state.mode == 'GUIDED'

        # Vision-loss failsafe: once the marker is lost the position estimate goes
        # stale, so the PID would hold against a frozen setpoint while the drone
        # drifts blind. Command LAND instead. One-shot; re-armed on fresh detection.
        if (is_armed and is_guided and self.last_aruco_time is not None
                and not self.vision_loss_landing):
            vision_age = (self.get_clock().now() - self.last_aruco_time).nanoseconds / 1e9
            if vision_age > self.vision_loss_timeout:
                self.vision_loss_landing = True
                self.get_logger().error('=' * 60)
                self.get_logger().error(
                    f'VISION LOST for {vision_age:.1f}s (>{self.vision_loss_timeout}s) - commanding LAND'
                )
                self.get_logger().error('=' * 60)
                self.set_mode('LAND')
                return

        # Reset PID timing when not active so the first active cycle doesn't get a huge dt
        if not (is_armed and is_guided and self.target_pose is not None):
            self.pid_last_time = None
            return

        if self.current_pose is None:
            # No position estimate yet; hold last setpoint via a pure position command
            self.setpoint_position_pub.publish(self.target_pose)
            return

        now = self.get_clock().now()
        if self.pid_last_time is None:
            self.pid_last_time = now
            return

        dt = (now - self.pid_last_time).nanoseconds / 1e9
        self.pid_last_time = now

        # Ignore degenerate dt (e.g. clock jump, first tick)
        if dt <= 0.0 or dt > 0.5:
            return

        # --- XY error in world frame ---
        error = np.array([
            self.target_pose.pose.position.x - self.current_pose.pose.position.x,
            self.target_pose.pose.position.y - self.current_pose.pose.position.y,
        ])

        # --- PID terms ---
        # Integral with anti-windup clamp (prevents integrator from accumulating
        # during periods when the drone cannot respond, e.g. mechanical limits)
        self.pid_integral += error * dt
        self.pid_integral = np.clip(
            self.pid_integral, -self.pid_integral_max, self.pid_integral_max
        )

        derivative = (error - self.pid_prev_error) / dt
        self.pid_prev_error = error.copy()

        vel_cmd = (
            self.pid_kp * error +
            self.pid_ki * self.pid_integral +
            self.pid_kd * derivative
        )
        vel_cmd = np.clip(vel_cmd, -self.pid_max_vel, self.pid_max_vel)

        # Deadband: zero out tiny velocity commands to prevent jitter at rest
        dist_xy = float(np.linalg.norm(error))
        if dist_xy < self.position_deadband:
            vel_cmd = np.zeros(2)

        msg = PositionTarget()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = 'map'
        msg.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        msg.type_mask = self._setpoint_type_mask
        msg.velocity.x = float(vel_cmd[0])
        msg.velocity.y = float(vel_cmd[1])
        msg.position.z = float(self.target_pose.pose.position.z)

        self.setpoint_raw_pub.publish(msg)

        # Diagnostics
        dist_3d = self._calculate_distance(self.current_pose, self.target_pose)
        self.get_logger().info(
            f'PID | Current: X={self.current_pose.pose.position.x:+.3f} '
            f'Y={self.current_pose.pose.position.y:+.3f} | '
            f'Target: X={self.target_pose.pose.position.x:+.3f} '
            f'Y={self.target_pose.pose.position.y:+.3f} | '
            f'Error: X={error[0]:+.3f} Y={error[1]:+.3f} | '
            f'Vel_cmd: X={vel_cmd[0]:+.3f} Y={vel_cmd[1]:+.3f} | '
            f'Integral: X={self.pid_integral[0]:+.3f} Y={self.pid_integral[1]:+.3f} | '
            f'Dist={dist_3d:.3f}m',
            throttle_duration_sec=2.0
        )

        if dist_3d < self.position_tolerance:
            self.get_logger().info(
                f'Target reached (dist={dist_3d:.3f}m)',
                throttle_duration_sec=2.0
            )
    
    def arm(self):
        """Arm the drone"""
        is_armed = self.mavros_state is not None and self.mavros_state.armed

        if not is_armed:
            self.get_logger().info('Arming drone...')
            req = CommandBool.Request()
            req.value = True

            future = self.arming_client.call_async(req)
            future.add_done_callback(self._arm_callback)
        else:
            self.get_logger().info('Drone already armed')

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
        is_armed = self.mavros_state is not None and self.mavros_state.armed

        if is_armed:
            self.get_logger().info('Disarming drone...')
            req = CommandBool.Request()
            req.value = False

            future = self.arming_client.call_async(req)
            future.add_done_callback(self._disarm_callback)
        else:
            self.get_logger().info('Drone already disarmed')

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

        # Set target position at current location + altitude
        # This ensures setpoints are published continuously
        if self.current_pose is not None:
            target = PoseStamped()
            target.header.stamp = self.get_clock().now().to_msg()
            target.header.frame_id = 'map'
            target.pose.position.x = self.current_pose.pose.position.x
            target.pose.position.y = self.current_pose.pose.position.y
            target.pose.position.z = altitude
            target.pose.orientation = self.current_pose.pose.orientation
            self.target_pose = target
        else:
            # If no position available, set target at origin + altitude
            target = PoseStamped()
            target.header.stamp = self.get_clock().now().to_msg()
            target.header.frame_id = 'map'
            target.pose.position.x = 0.0
            target.pose.position.y = 0.0
            target.pose.position.z = altitude
            target.pose.orientation.w = 1.0
            self.target_pose = target

        # Then arm if not already armed
        is_armed = self.mavros_state is not None and self.mavros_state.armed
        if not is_armed:
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
        # Reset PID so integral from old target doesn't pull toward wrong position
        self.pid_integral = np.zeros(2)
        self.pid_prev_error = np.zeros(2)
        self.pid_last_time = None

    def _calculate_distance(self, pose1: PoseStamped, pose2: PoseStamped) -> float:
        """Calculate Euclidean distance between two poses"""
        dx = pose1.pose.position.x - pose2.pose.position.x
        dy = pose1.pose.position.y - pose2.pose.position.y
        dz = pose1.pose.position.z - pose2.pose.position.z
        return np.sqrt(dx**2 + dy**2 + dz**2)

    def _manage_rc_override(self):
        """
        Manage RC override based on flight mode.
        In GUIDED mode: Override RC channels to 0 to disable RC input
        In other modes: Release override (65535 = no override)

        This ensures RC sticks don't interfere with autonomous control.
        Safety: Mode switch can still change flight modes.
        """
        if self.mavros_state is None:
            return

        is_guided = self.mavros_state.mode == 'GUIDED'
        is_armed = self.mavros_state.armed

        # Create RC override message
        msg = OverrideRCIn()
        msg.channels = [65535] * 18  # Default: no override

        if is_guided and is_armed:
            # In GUIDED mode: Override sticks to 0 (disable RC input)
            # Channels: 0=roll, 1=pitch, 2=throttle, 3=yaw
            msg.channels[0] = 0  # Roll
            msg.channels[1] = 0  # Pitch
            msg.channels[2] = 0  # Throttle - CRITICAL: disable throttle stick
            msg.channels[3] = 0  # Yaw
            # Note: Channel 4+ (mode switch, aux switches) remain 65535 (not overridden)
            # This allows pilot to still change flight modes for safety

            # Log when we first enter this state
            if self.last_mode != 'GUIDED':
                self.get_logger().info('GUIDED mode: RC sticks disabled (mode switch still active)')
        else:
            # In other modes: Release all overrides
            # All channels = 65535 means "don't override, use RC input"
            if self.last_mode == 'GUIDED' and self.mavros_state.mode != 'GUIDED':
                self.get_logger().info(f'{self.mavros_state.mode} mode: RC sticks re-enabled')

        self.last_mode = self.mavros_state.mode

        # Publish RC override
        self.rc_override_pub.publish(msg)


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
