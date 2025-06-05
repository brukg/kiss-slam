#!/usr/bin/env python3

import os
import time
from pathlib import Path as FilePath
from typing import Optional

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import PointCloud2, PointField
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry, Path
from visualization_msgs.msg import MarkerArray, Marker
from std_msgs.msg import Header
import tf2_ros
from tf2_geometry_msgs import PoseStamped as TF2PoseStamped

from kiss_slam.config import load_config
from kiss_slam.slam import KissSLAM


def ros_numpy_point_cloud(msg):
    """Convert ROS PointCloud2 message to numpy array."""
    import struct
    
    # Check if we have valid data
    if not msg.data:
        return np.array([]).reshape(0, 3)
    
    # Find x, y, z field offsets
    x_offset = y_offset = z_offset = None
    for field in msg.fields:
        if field.name == 'x':
            x_offset = field.offset
        elif field.name == 'y':
            y_offset = field.offset
        elif field.name == 'z':
            z_offset = field.offset
    
    if x_offset is None or y_offset is None or z_offset is None:
        raise ValueError("PointCloud2 message missing x, y, or z fields")
    
    # Extract points from PointCloud2 message
    points = []
    point_step = msg.point_step
    num_points = len(msg.data) // point_step
    
    for i in range(num_points):
        offset = i * point_step
        try:
            x = struct.unpack_from('f', msg.data, offset + x_offset)[0]
            y = struct.unpack_from('f', msg.data, offset + y_offset)[0]
            z = struct.unpack_from('f', msg.data, offset + z_offset)[0]
            
            # Skip invalid points (NaN or infinite values)
            if np.isfinite(x) and np.isfinite(y) and np.isfinite(z):
                points.append([x, y, z])
        except struct.error:
            continue
    
    return np.array(points, dtype=np.float64)


def numpy_to_pointcloud2(points, frame_id, stamp):
    """Convert numpy array to ROS PointCloud2 message."""
    import struct
    
    msg = PointCloud2()
    msg.header.frame_id = frame_id
    msg.header.stamp = stamp
    
    if len(points.shape) == 3:
        msg.height = points.shape[1]
        msg.width = points.shape[0]
    else:
        msg.height = 1
        msg.width = points.shape[0]
    
    msg.fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    msg.is_bigendian = False
    msg.point_step = 12
    msg.row_step = msg.point_step * msg.width
    msg.is_dense = True
    
    buffer = []
    for point in points:
        buffer.extend(struct.pack('fff', point[0], point[1], point[2]))
    msg.data = bytes(buffer)
    
    return msg


class KissSlamNode(Node):
    def __init__(self):
        super().__init__('kiss_slam_node')
        
        # Declare parameters
        self.declare_parameter('input_topic', '/points')
        self.declare_parameter('output_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('lidar_frame', 'lidar_link')
        self.declare_parameter('config_file', '')
        self.declare_parameter('visualize', True)
        self.declare_parameter('publish_rate', 10.0)
        
        # Get parameters
        self.input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        self.output_frame = self.get_parameter('output_frame').get_parameter_value().string_value
        self.base_frame = self.get_parameter('base_frame').get_parameter_value().string_value
        self.lidar_frame = self.get_parameter('lidar_frame').get_parameter_value().string_value
        config_file = self.get_parameter('config_file').get_parameter_value().string_value
        self.visualize = self.get_parameter('visualize').get_parameter_value().bool_value
        self.publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value
        
        # Initialize KISS-SLAM with error handling
        try:
            self.get_logger().info(f'Config file parameter: {config_file} (type: {type(config_file)})')
            
            # Handle config file path properly
            if config_file and str(config_file).strip():
                config_path = FilePath(str(config_file))
                self.get_logger().info(f'Loading KISS-SLAM config from: {config_path}')
            else:
                config_path = None
                self.get_logger().info('Using default KISS-SLAM configuration')
                
            self.slam_config = load_config(config_path)
            self.get_logger().info('Creating KissSLAM instance...')
            self.kiss_slam = KissSLAM(self.slam_config)
            self.get_logger().info('KissSLAM instance created successfully')
        except Exception as e:
            self.get_logger().error(f'Failed to initialize KISS-SLAM: {str(e)}')
            import traceback
            self.get_logger().error(traceback.format_exc())
            raise
        
        # Initialize ROS components
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        
        # Create subscriber
        self.pc_subscriber = self.create_subscription(
            PointCloud2,
            self.input_topic,
            self.pointcloud_callback,
            10
        )
        
        # Create publishers
        self.odom_publisher = self.create_publisher(Odometry, '/kiss_slam/odometry', 10)
        self.path_publisher = self.create_publisher(Path, '/kiss_slam/path', 10)
        self.pose_publisher = self.create_publisher(PoseStamped, '/kiss_slam/pose', 10)
        
        if self.visualize:
            self.map_publisher = self.create_publisher(PointCloud2, '/kiss_slam/local_map', 10)
            self.closure_publisher = self.create_publisher(MarkerArray, '/kiss_slam/loop_closures', 10)
        
        # Initialize path message
        self.path_msg = Path()
        self.path_msg.header.frame_id = self.output_frame
        
        # Processing statistics
        self.scan_count = 0
        self.total_time = 0.0
        
        # Timer for publishing results
        self.publish_timer = self.create_timer(1.0 / self.publish_rate, self.publish_results)
        
        self.get_logger().info(f'KISS-SLAM ROS2 node initialized')
        self.get_logger().info(f'Subscribing to: {self.input_topic}')
        self.get_logger().info(f'Output frame: {self.output_frame}')
        self.get_logger().info(f'Base frame: {self.base_frame}')
        
    def pointcloud_callback(self, msg):
        """Process incoming point cloud data."""
        try:
            start_time = time.perf_counter()
            
            # Convert ROS message to numpy array
            points = ros_numpy_point_cloud(msg)
            
            if points.shape[0] == 0:
                self.get_logger().warn('Received empty point cloud')
                return
            
            # Transform point cloud to base frame if necessary
            if msg.header.frame_id != self.base_frame:
                points = self.transform_pointcloud(points, msg.header.frame_id, msg.header.stamp)
                if points is None:
                    self.get_logger().warn('Failed to transform point cloud, skipping frame')
                    return
            
            # Process with KISS-SLAM
            # Create timestamps array for each point (KISS-SLAM expects per-point timestamps for deskewing)
            # For static point clouds, we use the same timestamp for all points
            timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            timestamps = np.full(points.shape[0], timestamp, dtype=np.float64)
            
            self.kiss_slam.process_scan(points, timestamps)
            
            # Update statistics
            self.scan_count += 1
            self.total_time += time.perf_counter() - start_time
            
            if self.scan_count % 10 == 0:  # More frequent logging initially
                avg_time = self.total_time / self.scan_count * 1000  # Convert to ms
                self.get_logger().info(f'Processed {self.scan_count} scans, avg time: {avg_time:.2f}ms')
                
        except Exception as e:
            self.get_logger().error(f'Error in pointcloud callback: {str(e)}')
            import traceback
            self.get_logger().error(traceback.format_exc())
    
    def transform_pointcloud(self, points, source_frame, stamp):
        """Transform point cloud from source frame to base frame."""
        try:
            # Get transform from source frame to base frame
            transform = self.tf_buffer.lookup_transform(
                self.base_frame,
                source_frame,
                rclpy.time.Time(),  # Use latest available#stamp,
                timeout=rclpy.duration.Duration(seconds=0.2)
            )
            
            self.get_logger().debug(f'Transforming {points.shape[0]} points from {source_frame} to {self.base_frame}')
            
            # Extract translation and rotation from transform
            translation = np.array([
                transform.transform.translation.x,
                transform.transform.translation.y,
                transform.transform.translation.z
            ])
            
            rotation = transform.transform.rotation
            
            # Convert quaternion to rotation matrix
            from scipy.spatial.transform import Rotation as R
            r = R.from_quat([rotation.x, rotation.y, rotation.z, rotation.w])
            rotation_matrix = r.as_matrix()
            
            # Create 4x4 transformation matrix
            T = np.eye(4)
            T[:3, :3] = rotation_matrix
            T[:3, 3] = translation
            
            # Transform points
            points_homogeneous = np.hstack([points, np.ones((points.shape[0], 1))])
            transformed_points = (T @ points_homogeneous.T).T
            
            return transformed_points[:, :3]  # Return only x, y, z
            
        except Exception as e:
            self.get_logger().error(f'Failed to transform point cloud: {str(e)}')
            return None
    
    def publish_results(self):
        """Publish SLAM results."""
        if self.scan_count == 0:
            return
            
        try:
            current_time = self.get_clock().now()
            
            # Get current pose from KISS-SLAM - but only if we've processed scans
            if self.scan_count == 0:
                return
                
            poses = self.kiss_slam.poses  # Use property, not method call
            if len(poses) == 0:
                self.get_logger().debug('No poses available yet')
                return
                
            current_pose_matrix = poses[-1]
            
            # Validate the pose matrix before using it
            if not np.all(np.isfinite(current_pose_matrix)):
                self.get_logger().warn('Invalid pose matrix (contains NaN/inf), skipping publication')
                return
            
            # Publish TF transform
            self.publish_transform(current_pose_matrix, current_time)
            
            # Publish odometry
            self.publish_odometry(current_pose_matrix, current_time)
            
            # Publish pose
            self.publish_pose(current_pose_matrix, current_time)
            
            # Publish path
            self.publish_path(current_pose_matrix, current_time)
            
            if self.visualize:
                # Publish local map
                self.publish_local_map(current_time)
                
                # Publish loop closures
                self.publish_loop_closures(current_time)
                
        except Exception as e:
            self.get_logger().error(f'Error publishing results: {str(e)}')
            import traceback
            self.get_logger().error(traceback.format_exc())
    
    def publish_transform(self, pose_matrix, stamp):
        """Publish TF transform."""
        transform = TransformStamped()
        transform.header.stamp = stamp.to_msg()
        transform.header.frame_id = self.output_frame
        transform.child_frame_id = self.base_frame
        
        # Extract translation
        transform.transform.translation.x = float(pose_matrix[0, 3])
        transform.transform.translation.y = float(pose_matrix[1, 3])
        transform.transform.translation.z = float(pose_matrix[2, 3])
        
        # Extract rotation (convert rotation matrix to quaternion)
        from scipy.spatial.transform import Rotation as R
        rotation = R.from_matrix(pose_matrix[:3, :3])
        quat = rotation.as_quat()  # Returns [x, y, z, w]
        
        transform.transform.rotation.x = float(quat[0])
        transform.transform.rotation.y = float(quat[1])
        transform.transform.rotation.z = float(quat[2])
        transform.transform.rotation.w = float(quat[3])
        
        self.tf_broadcaster.sendTransform(transform)
    
    def publish_odometry(self, pose_matrix, stamp):
        """Publish odometry message."""
        odom = Odometry()
        odom.header.stamp = stamp.to_msg()
        odom.header.frame_id = self.output_frame
        odom.child_frame_id = self.base_frame
        
        # Set pose
        odom.pose.pose.position.x = float(pose_matrix[0, 3])
        odom.pose.pose.position.y = float(pose_matrix[1, 3])
        odom.pose.pose.position.z = float(pose_matrix[2, 3])
        
        from scipy.spatial.transform import Rotation as R
        rotation = R.from_matrix(pose_matrix[:3, :3])
        quat = rotation.as_quat()
        
        odom.pose.pose.orientation.x = float(quat[0])
        odom.pose.pose.orientation.y = float(quat[1])
        odom.pose.pose.orientation.z = float(quat[2])
        odom.pose.pose.orientation.w = float(quat[3])
        
        # Set covariance (simple diagonal)
        odom.pose.covariance = [0.1] * 36
        odom.twist.covariance = [0.1] * 36
        
        self.odom_publisher.publish(odom)
    
    def publish_pose(self, pose_matrix, stamp):
        """Publish pose message."""
        pose = PoseStamped()
        pose.header.stamp = stamp.to_msg()
        pose.header.frame_id = self.output_frame
        
        pose.pose.position.x = float(pose_matrix[0, 3])
        pose.pose.position.y = float(pose_matrix[1, 3])
        pose.pose.position.z = float(pose_matrix[2, 3])
        
        from scipy.spatial.transform import Rotation as R
        rotation = R.from_matrix(pose_matrix[:3, :3])
        quat = rotation.as_quat()
        
        pose.pose.orientation.x = float(quat[0])
        pose.pose.orientation.y = float(quat[1])
        pose.pose.orientation.z = float(quat[2])
        pose.pose.orientation.w = float(quat[3])
        
        self.pose_publisher.publish(pose)
    
    def publish_path(self, pose_matrix, stamp):
        """Publish path message."""
        pose = PoseStamped()
        pose.header.stamp = stamp.to_msg()
        pose.header.frame_id = self.output_frame
        
        pose.pose.position.x = float(pose_matrix[0, 3])
        pose.pose.position.y = float(pose_matrix[1, 3])
        pose.pose.position.z = float(pose_matrix[2, 3])
        
        from scipy.spatial.transform import Rotation as R
        rotation = R.from_matrix(pose_matrix[:3, :3])
        quat = rotation.as_quat()
        
        pose.pose.orientation.x = float(quat[0])
        pose.pose.orientation.y = float(quat[1])
        pose.pose.orientation.z = float(quat[2])
        pose.pose.orientation.w = float(quat[3])
        
        self.path_msg.poses.append(pose)
        self.path_msg.header.stamp = stamp.to_msg()
        
        self.path_publisher.publish(self.path_msg)
    
    def publish_local_map(self, stamp):
        """Publish local map point cloud."""
        try:
            # Get current local map from KISS-SLAM voxel grid
            if hasattr(self.kiss_slam, 'voxel_grid'):
                points = self.kiss_slam.voxel_grid.point_cloud()
                if points.shape[0] > 0:
                    pc_msg = numpy_to_pointcloud2(points, self.output_frame, stamp.to_msg())
                    self.map_publisher.publish(pc_msg)
        except Exception as e:
            self.get_logger().debug(f'Could not publish local map: {str(e)}')
    
    def publish_loop_closures(self, stamp):
        """Publish loop closure visualization."""
        try:
            closures = self.kiss_slam.get_closures()
            if len(closures) > 0:
                marker_array = MarkerArray()
                
                key_poses = self.kiss_slam.get_keyposes()
                for i, closure in enumerate(closures):
                    idx1, idx2 = closure
                    if idx1 < len(key_poses) and idx2 < len(key_poses):
                        marker = Marker()
                        marker.header.frame_id = self.output_frame
                        marker.header.stamp = stamp.to_msg()
                        marker.ns = "loop_closures"
                        marker.id = i
                        marker.type = Marker.LINE_STRIP
                        marker.action = Marker.ADD
                        
                        marker.scale.x = 0.1
                        marker.color.r = 1.0
                        marker.color.g = 0.0
                        marker.color.b = 0.0
                        marker.color.a = 1.0
                        
                        # Add two points for the line
                        from geometry_msgs.msg import Point
                        p1 = Point()
                        p1.x = float(key_poses[idx1][0, 3])
                        p1.y = float(key_poses[idx1][1, 3])
                        p1.z = float(key_poses[idx1][2, 3])
                        
                        p2 = Point()
                        p2.x = float(key_poses[idx2][0, 3])
                        p2.y = float(key_poses[idx2][1, 3])
                        p2.z = float(key_poses[idx2][2, 3])
                        
                        marker.points = [p1, p2]
                        marker_array.markers.append(marker)
                
                self.closure_publisher.publish(marker_array)
        except Exception as e:
            self.get_logger().debug(f'Could not publish loop closures: {str(e)}')


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = KissSlamNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error: {e}')
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main() 