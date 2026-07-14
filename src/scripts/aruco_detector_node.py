#!/usr/bin/env python3
"""
ArUco Detector Node
Detects ArUco markers from camera feed and publishes their poses
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped, PoseArray
from std_msgs.msg import Header, Int32MultiArray
from cv_bridge import CvBridge
import cv2
import cv2.aruco as aruco
import numpy as np
from typing import Optional, Tuple, List


class ArucoDetectorNode(Node):
    """
    ROS2 node for detecting ArUco markers and estimating their poses
    """
    
    def __init__(self):
        super().__init__('aruco_detector_node')
        
        # Declare parameters
        self.declare_parameter('camera_topic', '/camera/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/camera_info')
        self.declare_parameter('marker_size', 0.15)  # Default ArUco marker size in meters
        self.declare_parameter('aruco_dict_type', 'DICT_4X4_50')
        self.declare_parameter('visualize', True)
        self.declare_parameter('publish_rate', 30.0)
        self.declare_parameter('distance_scale_factor', 1.0)  # Distance correction factor
        # Per-ID marker size overrides (parallel arrays — ROS2 params can't hold dicts).
        # Pose estimation scales linearly with the assumed marker dimension, so small
        # takeoff-pad markers (e.g. 3cm under the camera at rest) must declare their
        # true size or the reported distance is wrong by size_assumed / size_real.
        self.declare_parameter('marker_size_ids', [-1])      # marker IDs with a non-default size
        self.declare_parameter('marker_size_values', [0.0])  # matching sizes in meters

        # Get parameters
        camera_topic = self.get_parameter('camera_topic').value
        camera_info_topic = self.get_parameter('camera_info_topic').value
        self.marker_size = self.get_parameter('marker_size').value
        aruco_dict_name = self.get_parameter('aruco_dict_type').value
        self.visualize = self.get_parameter('visualize').value
        self.distance_scale_factor = self.get_parameter('distance_scale_factor').value
        self.marker_sizes = self._build_marker_size_map()
        
        # Initialize CV Bridge
        self.bridge = CvBridge()
        
        # Initialize ArUco dictionary and parameters
        self.aruco_dict = self._get_aruco_dict(aruco_dict_name)
        self.aruco_params = aruco.DetectorParameters()
        
        # Camera calibration parameters (will be updated from camera_info)
        self.camera_matrix: Optional[np.ndarray] = None
        self.dist_coeffs: Optional[np.ndarray] = None
        self.camera_info_received = False
        
        # Subscribers
        self.image_sub = self.create_subscription(
            Image,
            camera_topic,
            self.image_callback,
            10
        )
        
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            camera_info_topic,
            self.camera_info_callback,
            10
        )
        
        # Publishers
        self.pose_array_pub = self.create_publisher(
            PoseArray,
            '/aruco/poses',
            10
        )

        self.marker_ids_pub = self.create_publisher(
            Int32MultiArray,
            '/aruco/ids',
            10
        )
        
        self.detection_image_pub = self.create_publisher(
            Image,
            '/aruco/detection_image',
            10
        )
        
        # Store latest detections for other nodes
        self.latest_detections: List[Tuple[int, PoseStamped]] = []
        
        self.get_logger().info('ArUco Detector Node initialized')
        self.get_logger().info(f'Listening to camera topic: {camera_topic}')
        self.get_logger().info(f'Default marker size: {self.marker_size}m')
        if self.marker_sizes:
            overrides = ', '.join(f'ID {mid}: {size}m' for mid, size in sorted(self.marker_sizes.items()))
            self.get_logger().info(f'Per-ID marker size overrides: {overrides}')
        self.get_logger().info(f'ArUco dictionary: {aruco_dict_name}')
        self.get_logger().info(f'Distance scale factor: {self.distance_scale_factor:.3f}')

    def _build_marker_size_map(self) -> dict:
        """Build {marker_id: size_m} from the parallel override parameters"""
        ids = self.get_parameter('marker_size_ids').value or []
        values = self.get_parameter('marker_size_values').value or []
        # [-1] / [0.0] are the "no overrides" placeholders
        pairs = [(int(i), float(v)) for i, v in zip(ids, values) if i >= 0 and v > 0.0]
        if len(ids) != len(values):
            self.get_logger().warn(
                f'marker_size_ids ({len(ids)}) and marker_size_values ({len(values)}) '
                f'length mismatch — extra entries ignored'
            )
        return dict(pairs)

    def _get_marker_size(self, marker_id: int) -> float:
        """Physical size for a marker ID, falling back to the global default"""
        return self.marker_sizes.get(marker_id, self.marker_size)

    def _get_aruco_dict(self, dict_name: str):
        """Get ArUco dictionary from string name"""
        aruco_dict_map = {
            'DICT_4X4_50': aruco.DICT_4X4_50,
            'DICT_4X4_100': aruco.DICT_4X4_100,
            'DICT_4X4_250': aruco.DICT_4X4_250,
            'DICT_4X4_1000': aruco.DICT_4X4_1000,
            'DICT_5X5_50': aruco.DICT_5X5_50,
            'DICT_5X5_100': aruco.DICT_5X5_100,
            'DICT_5X5_250': aruco.DICT_5X5_250,
            'DICT_5X5_1000': aruco.DICT_5X5_1000,
            'DICT_6X6_50': aruco.DICT_6X6_50,
            'DICT_6X6_100': aruco.DICT_6X6_100,
            'DICT_6X6_250': aruco.DICT_6X6_250,
            'DICT_6X6_1000': aruco.DICT_6X6_1000,
        }
        
        if dict_name not in aruco_dict_map:
            self.get_logger().warn(f'Unknown ArUco dictionary: {dict_name}, using DICT_4X4_50')
            dict_name = 'DICT_4X4_50'
        
        return aruco.getPredefinedDictionary(aruco_dict_map[dict_name])
    
    def camera_info_callback(self, msg: CameraInfo):
        """Callback for camera calibration info"""
        if not self.camera_info_received:
            camera_matrix = np.array(msg.k).reshape(3, 3)

            # Reject uncalibrated cameras (zero focal length). The GoPro profile
            # ships with a placeholder calibration file that is all zeros, so a
            # missed calibration fails loudly on the bench instead of feeding
            # garbage poses to the EKF in the air.
            if camera_matrix[0, 0] <= 0.0 or camera_matrix[1, 1] <= 0.0:
                self.get_logger().error(
                    'camera_info has zero focal length — camera is NOT calibrated '
                    '(placeholder calibration file?). Refusing it; no poses will be '
                    'published. Calibrate the camera and update its calibration YAML '
                    '(see GOPRO_HERO4_SETUP.md).',
                    throttle_duration_sec=5.0
                )
                return

            self.camera_matrix = camera_matrix
            self.dist_coeffs = np.array(msg.d)
            self.camera_info_received = True

            # Log detailed calibration info
            fx = self.camera_matrix[0, 0]
            fy = self.camera_matrix[1, 1]
            cx = self.camera_matrix[0, 2]
            cy = self.camera_matrix[1, 2]

            self.get_logger().info('=' * 60)
            self.get_logger().info('Camera Calibration Info Received')
            self.get_logger().info('=' * 60)
            self.get_logger().info(f'Image size: {msg.width} x {msg.height}')
            self.get_logger().info(f'Focal length: fx={fx:.2f}, fy={fy:.2f} pixels')
            self.get_logger().info(f'Principal point: cx={cx:.2f}, cy={cy:.2f}')
            self.get_logger().info(f'Marker size configured: {self.marker_size}m ({self.marker_size*100}cm)')
            self.get_logger().info('=' * 60)
    
    def image_callback(self, msg: Image):
        """Callback for processing camera images"""
        if not self.camera_info_received:
            self.get_logger().warn('Waiting for camera calibration info...', throttle_duration_sec=5.0)
            return
        
        try:
            # Convert ROS Image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Detect ArUco markers
            corners, ids, rejected = aruco.detectMarkers(
                cv_image,
                self.aruco_dict,
                parameters=self.aruco_params
            )
            
            # Process detections
            if ids is not None and len(ids) > 0:
                # Estimate pose per marker with its own physical size — pose scales
                # linearly with the assumed dimension, so mixed-size setups (small
                # takeoff pad + large flight grid) need per-ID sizes
                rvecs = []
                tvecs = []
                for i in range(len(ids)):
                    size = self._get_marker_size(ids[i][0])
                    rv, tv, _ = aruco.estimatePoseSingleMarkers(
                        [corners[i]],
                        size,
                        self.camera_matrix,
                        self.dist_coeffs
                    )
                    rvecs.append(rv[0])
                    tvecs.append(tv[0])


                # Create PoseArray message
                pose_array = PoseArray()
                pose_array.header = Header()
                pose_array.header.stamp = self.get_clock().now().to_msg()
                pose_array.header.frame_id = 'camera_frame'
                marker_ids_msg = Int32MultiArray()
                
                self.latest_detections = []
                
                for i in range(len(ids)):
                    marker_id = ids[i][0]
                    rvec = rvecs[i][0]
                    tvec = tvecs[i][0]

                    # Apply distance scale correction factor
                    tvec_corrected = tvec * self.distance_scale_factor

                    # Convert rotation vector to quaternion
                    rotation_matrix, _ = cv2.Rodrigues(rvec)
                    quaternion = self._rotation_matrix_to_quaternion(rotation_matrix)

                    # Create PoseStamped for this marker
                    pose = PoseStamped()
                    pose.header = pose_array.header
                    pose.pose.position.x = float(tvec_corrected[0])
                    pose.pose.position.y = float(tvec_corrected[1])
                    pose.pose.position.z = float(tvec_corrected[2])
                    pose.pose.orientation.x = quaternion[0]
                    pose.pose.orientation.y = quaternion[1]
                    pose.pose.orientation.z = quaternion[2]
                    pose.pose.orientation.w = quaternion[3]

                    pose_array.poses.append(pose.pose)
                    marker_ids_msg.data.append(int(marker_id))
                    self.latest_detections.append((marker_id, pose))

                    # Log detection with both raw and corrected distances
                    distance_raw = np.linalg.norm(tvec)
                    distance_corrected = np.linalg.norm(tvec_corrected)

                    # Extract yaw angle from rotation matrix for orientation diagnostics
                    import math
                    # For a downward-facing camera detecting ground markers,
                    # yaw indicates rotation around Z-axis (marker rotation on ground)
                    yaw_rad = math.atan2(rotation_matrix[1, 0], rotation_matrix[0, 0])
                    yaw_deg = math.degrees(yaw_rad)

                    if abs(self.distance_scale_factor - 1.0) > 0.01:
                        # Correction factor is active
                        self.get_logger().info(
                            f'Marker ID {marker_id}: RAW={distance_raw:.3f}m -> CORRECTED={distance_corrected:.3f}m '
                            f'| Yaw={yaw_deg:.1f}° (marker rotation)',
                            throttle_duration_sec=1.0
                        )
                    else:
                        # No correction
                        self.get_logger().info(
                            f'Detected marker ID {marker_id} at distance {distance_raw:.3f}m | Yaw={yaw_deg:.1f}°',
                            throttle_duration_sec=1.0
                        )
                
                # Publish IDs immediately before poses so subscribers usually see
                # the matching ID list before processing the PoseArray.
                self.marker_ids_pub.publish(marker_ids_msg)
                self.pose_array_pub.publish(pose_array)
                
                # Visualize if enabled
                if self.visualize:
                    self._visualize_detections(cv_image, corners, ids, rvecs, tvecs)
            
            else:
                # No markers detected
                if self.visualize:
                    self._publish_visualization(cv_image)
        
        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')
    
    def _visualize_detections(self, image, corners, ids, rvecs, tvecs):
        """Draw detected markers and axes on image"""
        import math

        # Draw detected markers
        aruco.drawDetectedMarkers(image, corners, ids)

        # Draw axes and orientation info for each marker
        for i in range(len(ids)):
            marker_id = ids[i][0]

            # Draw 3D axes on marker (RED=X, GREEN=Y, BLUE=Z)
            cv2.drawFrameAxes(
                image,
                self.camera_matrix,
                self.dist_coeffs,
                rvecs[i],
                tvecs[i],
                self._get_marker_size(marker_id) * 0.5
            )

            # Calculate yaw angle
            rotation_matrix, _ = cv2.Rodrigues(rvecs[i])
            yaw_rad = math.atan2(rotation_matrix[1, 0], rotation_matrix[0, 0])
            yaw_deg = math.degrees(yaw_rad)

            # Get marker center in image
            corner = corners[i][0]
            center_x = int(np.mean(corner[:, 0]))
            center_y = int(np.mean(corner[:, 1]))

            # Draw orientation info text
            distance = np.linalg.norm(tvecs[i]) * self.distance_scale_factor
            text = f"ID:{marker_id} D:{distance:.2f}m Yaw:{yaw_deg:.0f}deg"

            # Background rectangle for text
            (text_w, text_h), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 2)
            cv2.rectangle(image,
                         (center_x - 5, center_y - text_h - 10),
                         (center_x + text_w + 5, center_y - 5),
                         (0, 0, 0), -1)

            # Text
            cv2.putText(image, text,
                       (center_x, center_y - 8),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            # Draw orientation indicator arrow (shows marker "forward" direction)
            # Arrow points in direction of marker's X-axis (RED axis)
            arrow_length = 40
            arrow_end_x = int(center_x + arrow_length * math.cos(yaw_rad))
            arrow_end_y = int(center_y + arrow_length * math.sin(yaw_rad))
            cv2.arrowedLine(image, (center_x, center_y), (arrow_end_x, arrow_end_y),
                          (0, 0, 255), 3, tipLength=0.3)  # Red arrow

        self._publish_visualization(image)
    
    def _publish_visualization(self, image):
        """Publish visualization image"""
        try:
            viz_msg = self.bridge.cv2_to_imgmsg(image, encoding='bgr8')
            self.detection_image_pub.publish(viz_msg)
        except Exception as e:
            self.get_logger().error(f'Error publishing visualization: {str(e)}')
    
    def _rotation_matrix_to_quaternion(self, R: np.ndarray) -> np.ndarray:
        """Convert rotation matrix to quaternion [x, y, z, w]"""
        trace = np.trace(R)
        
        if trace > 0:
            s = 0.5 / np.sqrt(trace + 1.0)
            w = 0.25 / s
            x = (R[2, 1] - R[1, 2]) * s
            y = (R[0, 2] - R[2, 0]) * s
            z = (R[1, 0] - R[0, 1]) * s
        elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
            s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
            w = (R[2, 1] - R[1, 2]) / s
            x = 0.25 * s
            y = (R[0, 1] + R[1, 0]) / s
            z = (R[0, 2] + R[2, 0]) / s
        elif R[1, 1] > R[2, 2]:
            s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
            w = (R[0, 2] - R[2, 0]) / s
            x = (R[0, 1] + R[1, 0]) / s
            y = 0.25 * s
            z = (R[1, 2] + R[2, 1]) / s
        else:
            s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
            w = (R[1, 0] - R[0, 1]) / s
            x = (R[0, 2] + R[2, 0]) / s
            y = (R[1, 2] + R[2, 1]) / s
            z = 0.25 * s
        
        return np.array([x, y, z, w])


def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetectorNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
