#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    """Generate launch description for KISS-SLAM."""
    
    # Declare launch arguments
    input_topic_arg = DeclareLaunchArgument(
        'input_topic',
        default_value='/points',
        description='Input point cloud topic'
    )
    
    output_frame_arg = DeclareLaunchArgument(
        'output_frame',
        default_value='map',
        description='Output frame for SLAM results'
    )
    
    base_frame_arg = DeclareLaunchArgument(
        'base_frame',
        default_value='base_link',
        description='Base frame of the robot'
    )
    
    lidar_frame_arg = DeclareLaunchArgument(
        'lidar_frame',
        default_value='lidar',
        description='LiDAR sensor frame'
    )
    
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('kiss_slam'),
            'conservative_slam_config.yaml'
        ]),
        description='Path to KISS-SLAM configuration file'
    )
    
    visualize_arg = DeclareLaunchArgument(
        'visualize',
        default_value='true',
        description='Enable visualization outputs'
    )
    
    publish_rate_arg = DeclareLaunchArgument(
        'publish_rate',
        default_value='10.0',
        description='Rate for publishing SLAM results (Hz)'
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )
    
    # KISS-SLAM node
    kiss_slam_node = Node(
        package='kiss_slam',
        executable='kiss_slam_ros2_node.py',
        name='kiss_slam_node',
        output='screen',
        parameters=[{
            'input_topic': LaunchConfiguration('input_topic'),
            'output_frame': LaunchConfiguration('output_frame'),
            'base_frame': LaunchConfiguration('base_frame'),
            'lidar_frame': LaunchConfiguration('lidar_frame'),
            'config_file': LaunchConfiguration('config_file'),
            'visualize': LaunchConfiguration('visualize'),
            'publish_rate': LaunchConfiguration('publish_rate'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
        remappings=[
            ('/points', LaunchConfiguration('input_topic')),
        ]
    )
    
    return LaunchDescription([
        input_topic_arg,
        output_frame_arg,
        base_frame_arg,
        lidar_frame_arg,
        config_file_arg,
        visualize_arg,
        publish_rate_arg,
        use_sim_time_arg,
        kiss_slam_node,
    ]) 