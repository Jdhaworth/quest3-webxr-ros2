#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get package share directory
    pkg_share = get_package_share_directory('quest3_webxr_ros2')
    
    # Declare launch arguments
    host_arg = DeclareLaunchArgument(
        'host',
        default_value='0.0.0.0',
        description='Host address for WebXR server'
    )
    
    http_port_arg = DeclareLaunchArgument(
        'http_port',
        default_value='8443',
        description='HTTP port for WebXR app'
    )
    
    ws_port_arg = DeclareLaunchArgument(
        'ws_port',
        default_value='8080',
        description='WebSocket port for communication'
    )
    
    wss_port_arg = DeclareLaunchArgument(
        'wss_port',
        default_value='8444',
        description='Secure WebSocket port for communication'
    )
    
    # Quest 3 WebXR Server node
    quest3_webxr_server_node = Node(
        package='quest3_webxr_ros2',
        executable='quest3_webxr_server',
        name='quest3_webxr_server',
        output='screen',
        parameters=[{
            'host': LaunchConfiguration('host'),
            'http_port': LaunchConfiguration('http_port'),
            'ws_port': LaunchConfiguration('ws_port'),
            'wss_port': LaunchConfiguration('wss_port'),
        }]
    )
    
    return LaunchDescription([
        host_arg,
        http_port_arg,
        ws_port_arg,
        wss_port_arg,
        quest3_webxr_server_node,
    ])
