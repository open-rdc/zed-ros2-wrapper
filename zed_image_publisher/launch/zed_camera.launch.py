#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate ZED Camera launch description."""
    
    # Get package directory
    pkg_dir = get_package_share_directory('zed_image_publisher')
    
    # Default config file path
    default_config_file = os.path.join(pkg_dir, 'config', 'zed_config.yaml')
    
    # Launch arguments
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=default_config_file,
        description='Path to ZED camera configuration YAML file'
    )
    
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Node namespace'
    )
    
    # ZED Camera Node
    zed_camera_node = Node(
        package='zed_image_publisher',
        executable='zed_image_publisher',
        name='zed_camera_node',
        namespace=LaunchConfiguration('namespace'),
        parameters=[LaunchConfiguration('config_file')],
        output='screen',
        emulate_tty=True,
    )
    
    return LaunchDescription([
        config_file_arg,
        namespace_arg,
        zed_camera_node
    ])