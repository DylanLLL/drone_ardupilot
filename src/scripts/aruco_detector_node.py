#!/usr/bin/env python3
"""
ArUco Detector Node
Detects ArUco markers from camera feed and publishes their poses
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped, PoseArray
from std_msgs.msg import Header
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
        self.declare_parameter('marker_size', 0.15)  # ArUco marker size in meters
        self.declare_parameter('aruco_dict_type', 'DICT_4X4_50')
        self.declare_parameter('visualize', True)
        self.declare_parameter('publish_rate', 30.0)
        self.declare_parameter('distance_scale_factor', 1.0)  # Distance correction factor

        # Get parameters
        camera_topic = self.get_parameter('camera_topic').value
        camera_info_topic = self.get_parameter('camera_info_topic').value
        self.marker_size = self.get_parameter('marker_size').value
        aruco_dict_name = self.get_parameter('aruco_dict_type').value
        self.visualize = self.get_parameter('visualize').value
        self.distance_scale_factor = self.get_parameter('distance_scale_factor').value
        
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
        
        self.detection_image_pub = self.create_publisher(
            Image,
            '/aruco/detection_image',
            10
        )
        
        # Store latest detections for other nodes
        self.latest_detections: List[Tuple[int, PoseStamped]] = []
        
        self.get_logger().info('ArUco Detector Node initialized')
        self.get_logger().info(f'Listening to camera topic: {camera_topic}')
        self.get_logger().info(f'Marker size: {self.marker_size}m')
        self.get_logger().info(f'ArUco dictionary: {aruco_dict_name}')
    
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
            self.camera_matrix = np.array(msg.k).reshape(3, 3)
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
                # Estimate pose for each marker
                rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
                    corners,
                    self.marker_size,
                    self.camera_matrix,
                    self.dist_coeffs
                )
                
                # Create PoseArray message
                pose_array = PoseArray()
                pose_array.header = Header()
                pose_array.header.stamp = self.get_clock().now().to_msg()
                pose_array.header.frame_id = 'camera_frame'
                
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
                    self.latest_detections.append((marker_id, pose))

                    # Log detection with both raw and corrected distances
                    distance_raw = np.linalg.norm(tvec)
                    distance_corrected = np.linalg.norm(tvec_corrected)

                    if abs(self.distance_scale_factor - 1.0) > 0.01:
                        # Correction factor is active
                        self.get_logger().info(
                            f'Marker ID {marker_id}: RAW={distance_raw:.3f}m -> CORRECTED={distance_corrected:.3f}m (scale={self.distance_scale_factor:.3f})',
                            throttle_duration_sec=1.0
                        )
                    else:
                        # No correction
                        self.get_logger().info(
                            f'Detected marker ID {marker_id} at distance {distance_raw:.3f}m',
                            throttle_duration_sec=1.0
                        )
                
                # Publish pose array
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
        # Draw detected markers
        aruco.drawDetectedMarkers(image, corners, ids)
        
        # Draw axes for each marker
        for i in range(len(ids)):
            cv2.drawFrameAxes(
                image,
                self.camera_matrix,
                self.dist_coeffs,
                rvecs[i],
                tvecs[i],
                self.marker_size * 0.5
            )
        
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
