#!/usr/bin/env python3
"""
Flight Test Recorder

Captures compact flight-test evidence for later debugging:
- topic snapshots as CSV
- image brightness/blur metrics for camera quality
- ArUco detection IDs/count and stale-vision age
- MAVROS state and selected ArduPilot parameters
- a Markdown summary suitable for sharing with an AI reviewer
"""

import csv
import math
import os
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Optional

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseArray, PoseStamped
from mavros_msgs.msg import PositionTarget, State
from mavros_msgs.srv import ParamGet
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int32MultiArray


class FlightTestRecorder(Node):
    """Record low-bandwidth diagnostics during bench and flight tests."""

    DEFAULT_PARAMS = [
        'EK3_SRC1_POSXY',
        'EK3_SRC1_POSZ',
        'EK3_SRC1_VELXY',
        'EK3_SRC1_YAW',
        'VISO_TYPE',
        'GPS_TYPE',
        'PSC_POSXY_P',
        'PSC_VELXY_P',
        'PSC_VELXY_I',
        'PSC_VELXY_D',
        'PSC_VELXY_FILT_HZ',
        'MOT_THST_HOVER',
        'MOT_HOVER_LEARN',
    ]

    def __init__(self):
        super().__init__('flight_test_recorder')

        self.declare_parameter('log_directory', '~/drone_test_logs')
        self.declare_parameter('session_name', '')
        self.declare_parameter('sample_rate', 5.0)
        self.declare_parameter('image_metrics_rate', 2.0)
        self.declare_parameter('param_capture_delay', 5.0)
        self.declare_parameter('camera_topic', '/camera/image_raw')
        self.declare_parameter('tracked_params', self.DEFAULT_PARAMS)

        log_directory = self.get_parameter('log_directory').value
        session_name = self.get_parameter('session_name').value
        sample_rate = float(self.get_parameter('sample_rate').value)
        self.image_metrics_period = 1.0 / float(self.get_parameter('image_metrics_rate').value)
        param_capture_delay = float(self.get_parameter('param_capture_delay').value)
        camera_topic = self.get_parameter('camera_topic').value
        self.tracked_params = list(self.get_parameter('tracked_params').value)

        self.session_started_at = datetime.now()
        session_stamp = self.session_started_at.strftime('%Y%m%d_%H%M%S')
        safe_session_name = self._safe_name(session_name) if session_name else f'flight_test_{session_stamp}'
        self.session_dir = Path(os.path.expanduser(log_directory)) / safe_session_name
        self.session_dir.mkdir(parents=True, exist_ok=True)

        self.bridge = CvBridge()
        self.sample_index = 0
        self.last_image_metric_time = None

        self.latest_state: Optional[State] = None
        self.latest_local_pose: Optional[PoseStamped] = None
        self.latest_vision_pose: Optional[PoseStamped] = None
        self.latest_odometry: Optional[Odometry] = None
        self.latest_setpoint_raw: Optional[PositionTarget] = None
        self.latest_aruco_poses: Optional[PoseArray] = None
        self.latest_aruco_ids: List[int] = []
        self.last_aruco_detection_time = None

        self.image_metrics = {
            'width': '',
            'height': '',
            'brightness_mean': '',
            'brightness_std': '',
            'blur_laplacian_var': '',
            'dark_pixel_pct': '',
            'bright_pixel_pct': '',
        }

        self.param_results: Dict[str, str] = {}
        self.param_client = self.create_client(ParamGet, '/mavros/param/get')

        self.csv_path = self.session_dir / 'samples.csv'
        self.summary_path = self.session_dir / 'summary.md'
        self.params_path = self.session_dir / 'params.md'
        self.notes_path = self.session_dir / 'notes.md'

        self.csv_file = self.csv_path.open('w', newline='', encoding='utf-8')
        self.csv_writer = csv.DictWriter(self.csv_file, fieldnames=self._csv_fields())
        self.csv_writer.writeheader()

        self._write_initial_summary()
        self._write_notes_template()

        self.create_subscription(State, '/mavros/state', self.state_callback, 10)
        self.create_subscription(PoseStamped, '/drone/local_position', self.local_pose_callback, 10)
        self.create_subscription(PoseStamped, '/mavros/vision_pose/pose', self.vision_pose_callback, 10)
        self.create_subscription(Odometry, '/drone/odometry', self.odometry_callback, 10)
        self.create_subscription(PositionTarget, '/mavros/setpoint_raw/local', self.setpoint_raw_callback, 10)
        self.create_subscription(PoseArray, '/aruco/poses', self.aruco_poses_callback, 10)
        self.create_subscription(Int32MultiArray, '/aruco/ids', self.aruco_ids_callback, 10)
        self.create_subscription(Image, camera_topic, self.image_callback, 5)

        self.sample_timer = self.create_timer(1.0 / sample_rate, self.write_sample)
        self.param_timer = self.create_timer(param_capture_delay, self.capture_params_once)

        self.get_logger().info(f'Flight Test Recorder writing to {self.session_dir}')

    def _safe_name(self, value: str) -> str:
        clean = ''.join(ch if ch.isalnum() or ch in ('-', '_') else '_' for ch in value.strip())
        return clean or datetime.now().strftime('flight_test_%Y%m%d_%H%M%S')

    def _csv_fields(self) -> List[str]:
        return [
            'sample',
            'wall_time',
            'ros_time_sec',
            'mode',
            'armed',
            'connected',
            'local_x',
            'local_y',
            'local_z',
            'vision_x',
            'vision_y',
            'vision_z',
            'vision_age_sec',
            'aruco_count',
            'aruco_ids',
            'aruco_age_sec',
            'aruco_first_x',
            'aruco_first_y',
            'aruco_first_z',
            'setpoint_vx',
            'setpoint_vy',
            'setpoint_z',
            'odom_x',
            'odom_y',
            'odom_z',
            'image_width',
            'image_height',
            'brightness_mean',
            'brightness_std',
            'blur_laplacian_var',
            'dark_pixel_pct',
            'bright_pixel_pct',
        ]

    def _write_initial_summary(self):
        content = [
            '# Flight Test Summary',
            '',
            f'- Session started: {self.session_started_at.isoformat(timespec="seconds")}',
            f'- Log directory: `{self.session_dir}`',
            '',
            '## Files',
            '',
            '- `samples.csv`: time-series topic and image-quality metrics',
            '- `params.md`: MAVROS parameter snapshot',
            '- `notes.md`: fill this in after the test while details are fresh',
            '',
            '## What To Check',
            '',
            '- Low `brightness_mean` or low `blur_laplacian_var` points to dark/blurred camera input.',
            '- Rising `aruco_age_sec` means the marker is not being detected continuously.',
            '- Compare `local_x/local_y` drift against `aruco_count` and image metrics.',
            '- Confirm `EK3_SRC1_YAW`, `PSC_*`, and `MOT_*` values in `params.md`.',
            '',
        ]
        self.summary_path.write_text('\n'.join(content), encoding='utf-8')

    def _write_notes_template(self):
        content = [
            '# Test Notes',
            '',
            '## Environment',
            '',
            '- Location:',
            '- Lighting:',
            '- Marker size / ID:',
            '- Marker orientation:',
            '- Camera exposure/focus notes:',
            '- Battery:',
            '- Payload:',
            '',
            '## Flight Timeline',
            '',
            '- Arm time:',
            '- Takeoff time:',
            '- Drift direction:',
            '- Mode changes:',
            '- Landing reason:',
            '',
            '## Operator Observations',
            '',
            '-',
            '',
        ]
        self.notes_path.write_text('\n'.join(content), encoding='utf-8')

    def state_callback(self, msg: State):
        self.latest_state = msg

    def local_pose_callback(self, msg: PoseStamped):
        self.latest_local_pose = msg

    def vision_pose_callback(self, msg: PoseStamped):
        self.latest_vision_pose = msg

    def odometry_callback(self, msg: Odometry):
        self.latest_odometry = msg

    def setpoint_raw_callback(self, msg: PositionTarget):
        self.latest_setpoint_raw = msg

    def aruco_poses_callback(self, msg: PoseArray):
        self.latest_aruco_poses = msg
        if msg.poses:
            self.last_aruco_detection_time = self.get_clock().now()

    def aruco_ids_callback(self, msg: Int32MultiArray):
        self.latest_aruco_ids = [int(marker_id) for marker_id in msg.data]

    def image_callback(self, msg: Image):
        now = self.get_clock().now()
        if self.last_image_metric_time is not None:
            age = (now - self.last_image_metric_time).nanoseconds / 1e9
            if age < self.image_metrics_period:
                return
        self.last_image_metric_time = now

        try:
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            self.image_metrics = {
                'width': int(msg.width),
                'height': int(msg.height),
                'brightness_mean': float(np.mean(gray)),
                'brightness_std': float(np.std(gray)),
                'blur_laplacian_var': float(cv2.Laplacian(gray, cv2.CV_64F).var()),
                'dark_pixel_pct': float(np.mean(gray < 30) * 100.0),
                'bright_pixel_pct': float(np.mean(gray > 225) * 100.0),
            }
        except Exception as exc:
            self.get_logger().warn(f'Could not compute image metrics: {exc}', throttle_duration_sec=5.0)

    def write_sample(self):
        now = self.get_clock().now()
        row = {
            'sample': self.sample_index,
            'wall_time': datetime.now().isoformat(timespec='milliseconds'),
            'ros_time_sec': f'{now.nanoseconds / 1e9:.3f}',
        }
        self.sample_index += 1

        state = self.latest_state
        row.update({
            'mode': state.mode if state else '',
            'armed': state.armed if state else '',
            'connected': state.connected if state else '',
        })

        self._add_pose(row, 'local', self.latest_local_pose)
        self._add_pose(row, 'vision', self.latest_vision_pose)
        self._add_odom_pose(row, self.latest_odometry)
        self._add_setpoint(row, self.latest_setpoint_raw)
        self._add_aruco(row, now)
        row.update({f'image_{key}': value for key, value in self.image_metrics.items()
                    if key in ('width', 'height')})
        for key in ('brightness_mean', 'brightness_std', 'blur_laplacian_var',
                    'dark_pixel_pct', 'bright_pixel_pct'):
            value = self.image_metrics.get(key, '')
            row[key] = self._fmt(value)

        self.csv_writer.writerow(row)
        self.csv_file.flush()

    def _add_pose(self, row: Dict[str, object], prefix: str, pose: Optional[PoseStamped]):
        if pose is None:
            row[f'{prefix}_x'] = ''
            row[f'{prefix}_y'] = ''
            row[f'{prefix}_z'] = ''
            if prefix == 'vision':
                row['vision_age_sec'] = ''
            return

        row[f'{prefix}_x'] = self._fmt(pose.pose.position.x)
        row[f'{prefix}_y'] = self._fmt(pose.pose.position.y)
        row[f'{prefix}_z'] = self._fmt(pose.pose.position.z)
        if prefix == 'vision':
            stamp_sec = pose.header.stamp.sec + pose.header.stamp.nanosec / 1e9
            now_sec = self.get_clock().now().nanoseconds / 1e9
            row['vision_age_sec'] = self._fmt(max(0.0, now_sec - stamp_sec))

    def _add_odom_pose(self, row: Dict[str, object], odom: Optional[Odometry]):
        if odom is None:
            row['odom_x'] = ''
            row['odom_y'] = ''
            row['odom_z'] = ''
            return
        row['odom_x'] = self._fmt(odom.pose.pose.position.x)
        row['odom_y'] = self._fmt(odom.pose.pose.position.y)
        row['odom_z'] = self._fmt(odom.pose.pose.position.z)

    def _add_setpoint(self, row: Dict[str, object], setpoint: Optional[PositionTarget]):
        if setpoint is None:
            row['setpoint_vx'] = ''
            row['setpoint_vy'] = ''
            row['setpoint_z'] = ''
            return
        row['setpoint_vx'] = self._fmt(setpoint.velocity.x)
        row['setpoint_vy'] = self._fmt(setpoint.velocity.y)
        row['setpoint_z'] = self._fmt(setpoint.position.z)

    def _add_aruco(self, row: Dict[str, object], now):
        poses = self.latest_aruco_poses.poses if self.latest_aruco_poses else []
        row['aruco_count'] = len(poses)
        row['aruco_ids'] = ' '.join(str(marker_id) for marker_id in self.latest_aruco_ids)

        if self.last_aruco_detection_time is None:
            row['aruco_age_sec'] = ''
        else:
            age = (now - self.last_aruco_detection_time).nanoseconds / 1e9
            row['aruco_age_sec'] = self._fmt(age)

        if poses:
            first = poses[0].position
            row['aruco_first_x'] = self._fmt(first.x)
            row['aruco_first_y'] = self._fmt(first.y)
            row['aruco_first_z'] = self._fmt(first.z)
        else:
            row['aruco_first_x'] = ''
            row['aruco_first_y'] = ''
            row['aruco_first_z'] = ''

    def _fmt(self, value):
        if value == '' or value is None:
            return ''
        if isinstance(value, (int, np.integer)):
            return int(value)
        if isinstance(value, (float, np.floating)):
            if not math.isfinite(float(value)):
                return ''
            return f'{float(value):.4f}'
        return value

    def capture_params_once(self):
        self.param_timer.cancel()

        if not self.param_client.wait_for_service(timeout_sec=2.0):
            self.params_path.write_text(
                '# Parameter Snapshot\n\n'
                '`/mavros/param/get` was not available. Capture parameters manually from Mission Planner.\n',
                encoding='utf-8'
            )
            self.get_logger().warn('MAVROS param service unavailable; wrote params.md placeholder')
            return

        self.param_results = {}
        self._request_param(0)

    def _request_param(self, index: int):
        if index >= len(self.tracked_params):
            self._write_params_file()
            return

        name = self.tracked_params[index]
        request = ParamGet.Request()
        request.param_id = name
        future = self.param_client.call_async(request)
        future.add_done_callback(lambda fut, idx=index, param=name: self._param_done(fut, idx, param))

    def _param_done(self, future, index: int, param_name: str):
        try:
            response = future.result()
            if response.success:
                self.param_results[param_name] = self._param_value_to_string(response.value)
            else:
                self.param_results[param_name] = 'UNAVAILABLE'
        except Exception as exc:
            self.param_results[param_name] = f'ERROR: {exc}'
        self._request_param(index + 1)

    def _param_value_to_string(self, value) -> str:
        real_value = float(value.real)
        integer_value = int(value.integer)
        if abs(real_value) > 1e-9:
            return f'{real_value:.6g}'
        return str(integer_value)

    def _write_params_file(self):
        lines = [
            '# Parameter Snapshot',
            '',
            f'- Captured: {datetime.now().isoformat(timespec="seconds")}',
            '',
            '| Parameter | Value |',
            '| --- | --- |',
        ]
        for param in self.tracked_params:
            lines.append(f'| `{param}` | `{self.param_results.get(param, "MISSING")}` |')
        lines.append('')
        self.params_path.write_text('\n'.join(lines), encoding='utf-8')
        self.get_logger().info(f'Wrote parameter snapshot to {self.params_path}')

    def destroy_node(self):
        try:
            self.csv_file.flush()
            self.csv_file.close()
        finally:
            super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = FlightTestRecorder()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Flight Test Recorder stopped')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
