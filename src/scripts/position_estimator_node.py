#!/usr/bin/env python3
"""
Position Estimator Node - Modified for Bootstrap
"""

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
    
    def __init__(self):
        super().__init__('position_estimator_node')
        
        # Declare parameters
        self.declare_parameter('marker_map_file', '')
        self.declare_parameter('use_vision_position', True)
        self.declare_parameter('publish_rate', 30.0)
        self.declare_parameter('min_marker_confidence', 0.7)

        # Get parameters
        marker_map_file = self.get_parameter('marker_map_file').value
        self.use_vision_position = self.get_parameter('use_vision_position').value
        publish_rate = self.get_parameter('publish_rate').value
        
        # Load marker map (ArUco ID -> world position)
        self.marker_map: Dict[int, np.ndarray] = {}
        if marker_map_file:
            self._load_marker_map(marker_map_file)
        
        # Current drone pose estimate
        self.current_pose: Optional[PoseStamped] = None
        self.pose_covariance = np.eye(6) * 0.1

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
            # For now, assume marker ID 0
            marker_id = 0
            
            if marker_id in self.marker_map:
                # Transform from camera to world frame using known marker position
                drone_position, drone_orientation = self._estimate_drone_position_from_marker(
                    marker_pose_camera,
                    self.marker_map[marker_id]
                )

                # Create pose estimate
                pose = PoseStamped()
                pose.header.stamp = self.get_clock().now().to_msg()
                pose.header.frame_id = 'map'
                pose.pose.position.x = drone_position[0]
                pose.pose.position.y = drone_position[1]
                pose.pose.position.z = drone_position[2]

                # Use correctly transformed orientation
                pose.pose.orientation.x = drone_orientation[0]
                pose.pose.orientation.y = drone_orientation[1]
                pose.pose.orientation.z = drone_orientation[2]
                pose.pose.orientation.w = drone_orientation[3]

                self.current_pose = pose
                
                # Mark vision as locked
                if not self.vision_locked:
                    self.vision_locked = True
                    self.get_logger().info('🔒 VISION LOCKED! Real position feedback active')
                
                self.get_logger().info(
                    f'Estimated position: [{drone_position[0]:.2f}, '
                    f'{drone_position[1]:.2f}, {drone_position[2]:.2f}]',
                    throttle_duration_sec=1.0
                )
            else:
                # Use camera-relative positioning if marker not in map
                self._use_camera_relative_pose(marker_pose_camera)
                
        except Exception as e:
            self.get_logger().error(f'Error estimating position: {str(e)}')
    
    def _estimate_drone_position_from_marker(
        self,
        marker_pose_camera: PoseStamped,
        marker_position_world: np.ndarray
    ) -> Tuple[np.ndarray, Tuple[float, float, float, float]]:
        """
        Estimate drone position AND orientation in world frame from marker detection.

        SIMPLIFIED approach for downward-facing camera detecting ground markers:
        - Markers are on ground (world Z = 0)
        - Camera points straight down
        - Camera Z distance = drone altitude
        - Camera X,Y offsets map to world X,Y offsets (with rotation correction)

        Returns:
            Tuple of (position_array, orientation_quaternion)
            - position_array: [x, y, z] in world frame
            - orientation_quaternion: (x, y, z, w) drone orientation in world frame
        """

        # Step 1: Extract marker offset in camera frame
        cam_x = marker_pose_camera.position.x
        cam_y = marker_pose_camera.position.y
        cam_z = marker_pose_camera.position.z  # Distance to marker (= altitude since marker on ground)

        # Step 2: Transform camera offsets to body/world frame
        # For camera facing DOWN (90° pitch):
        # - Camera +X (right in image) -> World +Y (right in world)
        # - Camera +Y (down in image) -> World -X (when camera rotated, image top points forward)
        # - Camera +Z (distance) -> Altitude above ground

        # Marker offset in world frame (relative to drone)
        # Camera sees marker at offset (cam_x, cam_y) from center
        # This means DRONE is offset by NEGATIVE of this from marker
        offset_x_world = -(-cam_y)  # Camera Y maps to -World X, then negate for drone offset = cam_y
        offset_y_world = -(cam_x)     # Camera X maps to World Y, then negate for drone offset = -cam_x

        # Step 3: Calculate drone world position
        # Marker is at marker_position_world
        # Drone is offset from marker by (offset_x_world, offset_y_world)
        # Altitude is cam_z (distance from camera to ground marker)
        drone_position = np.array([
            marker_position_world[0] + offset_x_world,  # X position
            marker_position_world[1] + offset_y_world,  # Y position
            cam_z                                        # Z = altitude (camera distance to ground)
        ])

        # Step 4: Orientation - for now, assume level flight (no roll/pitch)
        # TODO: Extract yaw from marker orientation if needed
        drone_orientation = (0.0, 0.0, 0.0, 1.0)  # Identity quaternion (no rotation)

        # Log for debugging
        self.get_logger().info(
            f'Camera: X={cam_x:.3f}m Y={cam_y:.3f}m Z={cam_z:.3f}m | Offset: dX={offset_x_world:.3f}m dY={offset_y_world:.3f}m',
            throttle_duration_sec=0.5
        )
        self.get_logger().info(
            f'Marker@[{marker_position_world[0]:.2f},{marker_position_world[1]:.2f},0] -> Drone@[{drone_position[0]:.3f},{drone_position[1]:.3f},{drone_position[2]:.3f}]',
            throttle_duration_sec=0.5
        )

        return drone_position, drone_orientation

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

    def _get_camera_to_body_transform(self) -> np.ndarray:
        """
        Get transformation matrix from camera frame to drone body frame.
        Camera is mounted facing straight down (90° pitch).

        Camera frame (OpenCV): X=right, Y=down, Z=forward
        When camera points DOWN and detects markers on GROUND:
          - Camera +Z (forward/depth) = distance to ground = drone altitude
          - Camera +X (right) maps to lateral offset
          - Camera +Y (down when camera faces forward, but we're pitched 90°)

        For downward-facing camera detecting ground markers:
          - Camera X -> Body Y (camera right = drone right)
          - Camera Y -> Body -X (camera down in image = drone forward when pitched)
          - Camera Z -> Altitude (distance down to ground)

        Body/World frame: X=forward (North), Y=right (East), Z=down

        Returns 4x4 homogeneous transformation matrix
        """
        # Camera to Body rotation matrix for 90° pitch down
        # When camera faces down, image top points forward
        R_body_camera = np.array([
            [ 0, -1,  0],   # Body X (forward) = -Camera Y (up in image)
            [ 1,  0,  0],   # Body Y (right) = Camera X (right in image)
            [ 0,  0,  1]    # Body Z (down) = Camera Z (depth/distance)
        ])

        # Create 4x4 homogeneous transformation
        T_body_camera = np.eye(4)
        T_body_camera[0:3, 0:3] = R_body_camera

        return T_body_camera

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

        # Check for vision timeout
        if self.vision_locked and self._check_vision_timeout():
            self.get_logger().warn('Vision lost! Holding last known position', throttle_duration_sec=2.0)
        
        try:
            # Publish to MAVROS vision_pose for position feedback
            if self.use_vision_position:
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