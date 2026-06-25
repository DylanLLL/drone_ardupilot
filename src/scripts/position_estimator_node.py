#!/usr/bin/env python3
"""
Position Estimator Node - Modified for Bootstrap
"""

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseArray, TransformStamped
from nav_msgs.msg import Odometry
from mavros_msgs.msg import State, PositionTarget
from mavros_msgs.srv import CommandBool, SetMode
import numpy as np
import yaml
from pathlib import Path
from typing import Dict, Optional, Tuple
import tf2_ros
from tf2_ros import TransformBroadcaster


class PositionEstimatorNode(Node):
    """
    Estimates drone position from ArUco markers and publishes to MAVROS
    Includes bootstrap mode for takeoff without initial vision
    """

    # Fixed rotation: camera frame -> drone body frame.
    # Camera mounted downward, image-top pointing toward drone nose:
    #   Camera +X (image right) = Body +Y
    #   Camera +Y (image down)  = Body -X
    #   Camera +Z (depth)       = Body +Z
    _R_BODY_CAM = np.array([[ 0.0, -1.0,  0.0],
                             [ 1.0,  0.0,  0.0],
                             [ 0.0,  0.0,  1.0]])

    def __init__(self):
        super().__init__('position_estimator_node')
        
        # Declare parameters
        self.declare_parameter('marker_map_file', 'src/maps/marker_map_single.yaml')
        self.declare_parameter('use_vision_position', True)
        self.declare_parameter('publish_rate', 30.0)
        self.declare_parameter('min_marker_confidence', 0.7)
        self.declare_parameter('position_filter_alpha', 0.6)  # 0.3=heavy smoothing, 0.7=light smoothing

        # Camera lever arm: position of the camera relative to the body/FC centre,
        # in body axes (X=forward, Y=right, Z=down), in metres. The vision formula
        # returns the CAMERA's world position; this offset corrects it to the drone
        # centre. A camera mounted forward of the FC needs camera_offset_forward > 0,
        # otherwise any yaw rotation sweeps the offset and the EKF sees a phantom
        # XY translation it will chase (drift). Measure with a ruler from the FC
        # centre to the camera lens. Leave at 0 if the camera sits over the centre.
        self.declare_parameter('camera_offset_forward', 0.0)
        self.declare_parameter('camera_offset_right', 0.0)
        self.declare_parameter('camera_offset_down', 0.0)

        # Get parameters
        marker_map_file = self.get_parameter('marker_map_file').value
        self.use_vision_position = self.get_parameter('use_vision_position').value
        publish_rate = self.get_parameter('publish_rate').value
        cam_offset = np.array([
            self.get_parameter('camera_offset_forward').value,
            self.get_parameter('camera_offset_right').value,
            self.get_parameter('camera_offset_down').value,
        ])
        # None = fast path (no correction) when the camera is centred
        self._cam_offset_body = cam_offset if np.any(cam_offset) else None
        self.filter_alpha = self.get_parameter('position_filter_alpha').value
        
        # Load marker map (ArUco ID -> world position)
        self.marker_map: Dict[int, np.ndarray] = {}
        if marker_map_file:
            self._load_marker_map(marker_map_file)
        
        # Current drone pose estimate
        self.current_pose: Optional[PoseStamped] = None
        self.pose_covariance = np.eye(6) * 0.1

        # Position filtering (exponential moving average for smoothing)
        self.filtered_position: Optional[np.ndarray] = None
        # Yaw filter uses separate sin/cos components to handle angle wrapping correctly
        self.filtered_yaw_sin: Optional[float] = None
        self.filtered_yaw_cos: Optional[float] = None
        # filter_alpha loaded from parameter above

        # Vision state
        self.vision_locked = False
        self.last_aruco_detection_time = None
        
        # MAVROS state
        self.mavros_state: Optional[State] = None
        self.is_armed = False
        self.current_mode = ""
        
        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Subscribers
        self.aruco_sub = self.create_subscription(
            PoseArray,
            '/aruco/poses',
            self.aruco_callback,
            10
        )
        
        self.mavros_state_sub = self.create_subscription(
            State,
            '/mavros/state',
            self.mavros_state_callback,
            10
        )
        
        # Publishers
        self.vision_pose_pub = self.create_publisher(
            PoseStamped,
            '/mavros/vision_pose/pose',
            10
        )
        
        self.odometry_pub = self.create_publisher(
            Odometry,
            '/drone/odometry',
            10
        )
        
        self.local_position_pub = self.create_publisher(
            PoseStamped,
            '/drone/local_position',
            10
        )
        
        # Timer for publishing position at fixed rate
        self.publish_timer = self.create_timer(
            1.0 / publish_rate,
            self.publish_position
        )
        
        self.get_logger().info('Position Estimator Node initialized')
        if self.marker_map:
            self.get_logger().info(f'Loaded {len(self.marker_map)} markers from map')
        else:
            self.get_logger().warn('No marker map loaded - using camera-relative positioning')
        self.get_logger().info(f'Position filter alpha: {self.filter_alpha:.2f} (lower=smoother, higher=more responsive)')
    
    def _load_marker_map(self, filepath: str):
        """Load ArUco marker positions from YAML file"""
        try:
            with open(filepath, 'r') as f:
                data = yaml.safe_load(f)
                
            if 'markers' in data:
                for marker in data['markers']:
                    marker_id = marker['id']
                    position = np.array([
                        marker['position']['x'],
                        marker['position']['y'],
                        marker['position']['z']
                    ])
                    self.marker_map[marker_id] = position
                    
                self.get_logger().info(f'Loaded marker map from {filepath}')
            else:
                self.get_logger().error('Invalid marker map format')
                
        except Exception as e:
            self.get_logger().error(f'Failed to load marker map: {str(e)}')
    
    def mavros_state_callback(self, msg: State):
        """Callback for MAVROS state updates"""
        self.mavros_state = msg
        self.is_armed = msg.armed
        self.current_mode = msg.mode
    
    def aruco_callback(self, msg: PoseArray):
        """Process ArUco detections and estimate position"""
        if not msg.poses:
            return
        
        # Update last detection time
        self.last_aruco_detection_time = self.get_clock().now()
        
        try:
            # For now, use the first detected marker
            marker_pose_camera = msg.poses[0]

            # TODO: Get marker ID from custom message
            # For now, try all markers in the map (works for single marker setup)
            # Assume the detected marker is one of our known markers
            marker_id = list(self.marker_map.keys())[0] if self.marker_map else 0

            if marker_id in self.marker_map:
                drone_position_raw, raw_yaw = self._estimate_drone_position_from_marker(
                    marker_pose_camera,
                    self.marker_map[marker_id]
                )

                # EMA filter on XYZ position
                if self.filtered_position is None:
                    self.filtered_position = drone_position_raw
                else:
                    self.filtered_position = (
                        self.filter_alpha * drone_position_raw +
                        (1 - self.filter_alpha) * self.filtered_position
                    )

                # Circular EMA filter on yaw (avoids wrap-around errors near ±π)
                new_sin = math.sin(raw_yaw)
                new_cos = math.cos(raw_yaw)
                if self.filtered_yaw_sin is None:
                    self.filtered_yaw_sin = new_sin
                    self.filtered_yaw_cos = new_cos
                else:
                    self.filtered_yaw_sin = self.filter_alpha * new_sin + (1 - self.filter_alpha) * self.filtered_yaw_sin
                    self.filtered_yaw_cos = self.filter_alpha * new_cos + (1 - self.filter_alpha) * self.filtered_yaw_cos
                filtered_yaw = math.atan2(self.filtered_yaw_sin, self.filtered_yaw_cos)
                half_yaw = filtered_yaw / 2.0
                drone_orientation = (0.0, 0.0, math.sin(half_yaw), math.cos(half_yaw))

                drone_position = self.filtered_position

                # Create pose estimate
                pose = PoseStamped()
                pose.header.stamp = self.get_clock().now().to_msg()
                pose.header.frame_id = 'map'
                pose.pose.position.x = float(drone_position[0])
                pose.pose.position.y = float(drone_position[1])
                pose.pose.position.z = float(drone_position[2])
                pose.pose.orientation.x = drone_orientation[0]
                pose.pose.orientation.y = drone_orientation[1]
                pose.pose.orientation.z = drone_orientation[2]
                pose.pose.orientation.w = drone_orientation[3]

                self.current_pose = pose

                # Mark vision as locked
                if not self.vision_locked:
                    self.vision_locked = True
                    self.get_logger().info('VISION LOCKED! Real position feedback active')

                self.get_logger().info(
                    f'Position - RAW: [{drone_position_raw[0]:+.3f}, {drone_position_raw[1]:+.3f}, {drone_position_raw[2]:+.3f}] | '
                    f'FILTERED: [{drone_position[0]:+.3f}, {drone_position[1]:+.3f}, {drone_position[2]:+.3f}] | '
                    f'Yaw: {math.degrees(filtered_yaw):+.1f}°',
                    throttle_duration_sec=1.0
                )
            else:
                # Use camera-relative positioning if marker not in map
                self._use_camera_relative_pose(marker_pose_camera)
                
        except Exception as e:
            self.get_logger().error(f'Error estimating position: {str(e)}')
    
    def _estimate_drone_position_from_marker(
        self,
        marker_pose_camera,
        marker_position_world: np.ndarray
    ) -> Tuple[np.ndarray, Tuple[float, float, float, float]]:
        """
        Estimate drone position and yaw in world frame from a detected ArUco marker.

        Uses the full rotation matrix from ArUco to correctly account for drone yaw.
        This replaces the original yaw-blind approach that caused lateral drift whenever
        the drone was not perfectly aligned with the marker's axes.

        Derivation:
          - ArUco gives R_cam_marker: transforms marker-frame vectors to camera-frame vectors
          - Drone world pos = marker_world - R_cam_marker^T @ cam_translation
          - Yaw is extracted by chaining R_cam_marker^T with the known camera-body mount rotation

        Assumes markers are placed with axes aligned to the world frame (marker X = world X).
        The camera is mounted facing straight down with image-top pointing toward drone nose.
        """
        cam_x = marker_pose_camera.position.x
        cam_y = marker_pose_camera.position.y
        cam_z = marker_pose_camera.position.z  # altitude: camera-to-ground distance

        # Build rotation matrix from the quaternion output of aruco_detector_node.
        # R_cam_marker transforms marker-frame vectors into camera-frame vectors.
        q = marker_pose_camera.orientation
        R_cam_marker = self._quaternion_to_rotation_matrix((q.x, q.y, q.z, q.w))

        # Drone position formula (derived from camera projection inverse):
        #   cam_translation = R_cam_marker @ (marker_world - drone_world)
        #   => drone_world = marker_world - R_cam_marker^T @ cam_translation
        cam_translation = np.array([cam_x, cam_y, cam_z])
        world_offset = R_cam_marker.T @ cam_translation

        drone_position = np.array([
            marker_position_world[0] - world_offset[0],
            marker_position_world[1] - world_offset[1],
            cam_z  # altitude = camera distance to ground (not rotated)
        ])

        # Camera-to-body rotation gives both yaw and the world-frame direction of
        # the camera lever arm.
        R_world_body = R_cam_marker.T @ self._R_BODY_CAM.T
        yaw = math.atan2(R_world_body[1, 0], R_world_body[0, 0])

        # Lever-arm correction: drone_position above is the CAMERA's world position.
        # Rotate the body-frame camera offset into the world frame and subtract it so
        # the reported position tracks the drone centre rather than the camera.
        if self._cam_offset_body is not None:
            world_lever = R_world_body @ self._cam_offset_body
            drone_position[0] -= world_lever[0]
            drone_position[1] -= world_lever[1]

        self.get_logger().info(
            f'Cam=({cam_x:.3f}, {cam_y:.3f}, {cam_z:.3f}) | '
            f'Drone=[{drone_position[0]:.3f}, {drone_position[1]:.3f}, {drone_position[2]:.3f}] | '
            f'Yaw_raw={math.degrees(yaw):.1f}°',
            throttle_duration_sec=0.5
        )

        return drone_position, yaw

    def _quaternion_to_rotation_matrix(self, quat: Tuple[float, float, float, float]) -> np.ndarray:
        """
        Convert quaternion [x, y, z, w] to 3x3 rotation matrix
        """
        x, y, z, w = quat

        # First row
        r00 = 1 - 2*(y*y + z*z)
        r01 = 2*(x*y - w*z)
        r02 = 2*(x*z + w*y)

        # Second row
        r10 = 2*(x*y + w*z)
        r11 = 1 - 2*(x*x + z*z)
        r12 = 2*(y*z - w*x)

        # Third row
        r20 = 2*(x*z - w*y)
        r21 = 2*(y*z + w*x)
        r22 = 1 - 2*(x*x + y*y)

        return np.array([[r00, r01, r02],
                         [r10, r11, r12],
                         [r20, r21, r22]])

    def _rotation_matrix_to_quaternion(self, R_mat: np.ndarray) -> Tuple[float, float, float, float]:
        """
        Convert 3x3 rotation matrix to quaternion [x, y, z, w]
        """
        trace = np.trace(R_mat)

        if trace > 0:
            s = 0.5 / np.sqrt(trace + 1.0)
            w = 0.25 / s
            x = (R_mat[2, 1] - R_mat[1, 2]) * s
            y = (R_mat[0, 2] - R_mat[2, 0]) * s
            z = (R_mat[1, 0] - R_mat[0, 1]) * s
        elif R_mat[0, 0] > R_mat[1, 1] and R_mat[0, 0] > R_mat[2, 2]:
            s = 2.0 * np.sqrt(1.0 + R_mat[0, 0] - R_mat[1, 1] - R_mat[2, 2])
            w = (R_mat[2, 1] - R_mat[1, 2]) / s
            x = 0.25 * s
            y = (R_mat[0, 1] + R_mat[1, 0]) / s
            z = (R_mat[0, 2] + R_mat[2, 0]) / s
        elif R_mat[1, 1] > R_mat[2, 2]:
            s = 2.0 * np.sqrt(1.0 + R_mat[1, 1] - R_mat[0, 0] - R_mat[2, 2])
            w = (R_mat[0, 2] - R_mat[2, 0]) / s
            x = (R_mat[0, 1] + R_mat[1, 0]) / s
            y = 0.25 * s
            z = (R_mat[1, 2] + R_mat[2, 1]) / s
        else:
            s = 2.0 * np.sqrt(1.0 + R_mat[2, 2] - R_mat[0, 0] - R_mat[1, 1])
            w = (R_mat[1, 0] - R_mat[0, 1]) / s
            x = (R_mat[0, 2] + R_mat[2, 0]) / s
            y = (R_mat[1, 2] + R_mat[2, 1]) / s
            z = 0.25 * s

        return (x, y, z, w)

    def _use_camera_relative_pose(self, marker_pose_camera):
        """Use camera-relative positioning when marker map not available"""
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = 'camera_frame'
        pose.pose = marker_pose_camera
        
        self.current_pose = pose
        
        if not self.vision_locked:
            self.vision_locked = True
            self.get_logger().info('🔒 VISION LOCKED (camera-relative)!')
    
    def _check_vision_timeout(self) -> bool:
        """Check if we've lost vision (no detections for >1 second)"""
        if self.last_aruco_detection_time is None:
            return True
        
        time_since_detection = (self.get_clock().now() - self.last_aruco_detection_time).nanoseconds / 1e9
        return time_since_detection > 1.0
    
    def publish_position(self):
        """Publish current position estimate to MAVROS and other topics"""

        # Only publish if we have a valid position estimate
        if self.current_pose is None:
            self.get_logger().warn(
                'No position estimate available - waiting for ArUco marker detection',
                throttle_duration_sec=2.0
            )
            return

        # Check for vision timeout. Critically: when vision is stale we must STOP
        # feeding the pose to ArduPilot. Republishing the frozen pose with fresh
        # timestamps tells the EKF "vision healthy, drone stationary" while the
        # drone physically drifts — masking the loss from ArduPilot's EKF failsafe
        # (the only protection in pilot modes like LOITER).
        vision_stale = self.vision_locked and self._check_vision_timeout()
        if vision_stale:
            self.get_logger().warn(
                'Vision lost! Withholding stale pose from MAVROS (EKF will coast)',
                throttle_duration_sec=2.0
            )

        try:
            # Publish to MAVROS vision_pose for position feedback (fresh vision only)
            if self.use_vision_position and not vision_stale:
                vision_pose = PoseStamped()
                vision_pose.header = self.current_pose.header
                vision_pose.header.stamp = self.get_clock().now().to_msg()
                vision_pose.pose = self.current_pose.pose
                self.vision_pose_pub.publish(vision_pose)
            
            # Publish local position estimate
            self.local_position_pub.publish(self.current_pose)
            
            # Publish odometry
            odom = Odometry()
            odom.header = self.current_pose.header
            odom.child_frame_id = 'base_link'
            odom.pose.pose = self.current_pose.pose
            
            # Set covariance
            odom.pose.covariance = list(self.pose_covariance.flatten())
            
            self.odometry_pub.publish(odom)
            
            # Broadcast TF
            self._broadcast_tf()
            
        except Exception as e:
            self.get_logger().error(f'Error publishing position: {str(e)}')

    def _broadcast_tf(self):
        """Broadcast transform from map to base_link"""
        if self.current_pose is None:
            return
        
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'base_link'
        
        t.transform.translation.x = self.current_pose.pose.position.x
        t.transform.translation.y = self.current_pose.pose.position.y
        t.transform.translation.z = self.current_pose.pose.position.z
        
        t.transform.rotation = self.current_pose.pose.orientation
        
        self.tf_broadcaster.sendTransform(t)
    
    def get_current_position(self) -> Optional[np.ndarray]:
        """Get current estimated position as numpy array"""
        if self.current_pose is None:
            return None
        
        return np.array([
            self.current_pose.pose.position.x,
            self.current_pose.pose.position.y,
            self.current_pose.pose.position.z
        ])


def main(args=None):
    rclpy.init(args=args)
    node = PositionEstimatorNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()