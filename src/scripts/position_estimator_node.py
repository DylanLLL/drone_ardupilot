#!/usr/bin/env python3
"""
Position Estimator Node
Converts ArUco marker detections to global position estimates
and publishes to ArduPilot via MAVROS
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
        self.pose_covariance = np.eye(6) * 0.1  # Initial covariance
        
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
        
        try:
            # For now, use the first detected marker
            # In future, implement multi-marker fusion
            marker_pose_camera = msg.poses[0]
            
            # TODO: Get marker ID from custom message
            # For now, assume marker ID 0
            marker_id = 0
            
            if marker_id in self.marker_map:
                # Transform from camera to world frame using known marker position
                drone_position = self._estimate_drone_position_from_marker(
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
                
                # For now, keep orientation from marker detection
                pose.pose.orientation = marker_pose_camera.orientation
                
                self.current_pose = pose
                
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
    ) -> np.ndarray:
        """
        Estimate drone position in world frame from marker detection
        
        Args:
            marker_pose_camera: Marker pose in camera frame
            marker_position_world: Known marker position in world frame
            
        Returns:
            Estimated drone position in world frame
        """
        # Extract translation from camera to marker
        t_camera_marker = np.array([
            marker_pose_camera.position.x,
            marker_pose_camera.position.y,
            marker_pose_camera.position.z
        ])
        
        # For ground-based markers, assume camera is pointing down
        # Transform camera-to-marker into world frame
        # This is a simplified transform - in production, use full tf2 transforms
        
        # Assuming camera is mounted facing down on drone
        # X-forward, Y-left, Z-down in camera frame
        # X-forward, Y-left, Z-up in world frame
        
        # Invert the transformation to get drone position
        # drone_position = marker_position - camera_offset
        drone_position = marker_position_world.copy()
        drone_position[0] -= t_camera_marker[0]  # X offset
        drone_position[1] -= t_camera_marker[1]  # Y offset
        drone_position[2] = abs(t_camera_marker[2])  # Altitude (positive up)
        
        return drone_position
    
    def _use_camera_relative_pose(self, marker_pose_camera):
        """Use camera-relative positioning when marker map not available"""
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = 'camera_frame'
        pose.pose = marker_pose_camera
        
        self.current_pose = pose
    
    def publish_position(self):
        """Publish current position estimate to MAVROS and other topics"""
        if self.current_pose is None:
            return
        
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
            
            # Set covariance (simplified)
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
