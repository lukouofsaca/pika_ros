#!/usr/bin/env python3
"""
Launch file for data capture node
This can be used with both ROS1 roslaunch and ROS2 ros2 launch commands
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for data capture"""
    
    # Declare launch arguments
    dataset_dir_arg = DeclareLaunchArgument(
        'dataset_dir',
        default_value='/home/agilex/data',
        description='Directory to save captured data'
    )
    
    episode_index_arg = DeclareLaunchArgument(
        'episode_index',
        default_value='0',
        description='Episode index for this capture session'
    )
    
    capture_type_arg = DeclareLaunchArgument(
        'type',
        default_value='aloha',
        description='Type of data capture configuration'
    )
    
    config_path_arg = DeclareLaunchArgument(
        'config_path',
        default_value='',
        description='Path to configuration YAML file'
    )
    
    # Data capture node
    data_capture_node = Node(
        package='data_tools',
        executable='data_capture_node.py',
        name='data_capture_node',
        output='screen',
        parameters=[{
            'dataset_dir': LaunchConfiguration('dataset_dir'),
            'episode_index': LaunchConfiguration('episode_index'),
            'type': LaunchConfiguration('type'),
            'config_path': LaunchConfiguration('config_path'),
        }]
    )
    
    return LaunchDescription([
        dataset_dir_arg,
        episode_index_arg,
        capture_type_arg,
        config_path_arg,
        data_capture_node,
    ])
