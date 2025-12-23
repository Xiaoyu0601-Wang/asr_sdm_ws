#!/usr/bin/env python3
"""
ROS2 Launch file for SVO Visual Odometry - Test Rig 3 configuration.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml


def generate_launch_description():
    # Get the package share directory
    svo_ros_dir = get_package_share_directory('svo_ros')
    
    # Load camera calibration parameters from YAML
    camera_yaml_path = os.path.join(svo_ros_dir, 'param', 'camera_atan.yaml')
    with open(camera_yaml_path, 'r') as f:
        camera_params = yaml.safe_load(f)
    
    # Load VO parameters from YAML
    vo_yaml_path = os.path.join(svo_ros_dir, 'param', 'vo_accurate2.yaml')
    with open(vo_yaml_path, 'r') as f:
        vo_params = yaml.safe_load(f)
    
    # Combine all parameters
    node_parameters = {
        # Camera topic to subscribe to
        'cam_topic': '/camera/image_raw',
        
        # Initial camera orientation, make it point downwards
        'init_rx': 3.14,
        'init_ry': 0.0,
        'init_rz': 0.0,
    }
    
    # Add camera calibration parameters
    node_parameters.update(camera_params)
    
    # Add VO parameters
    node_parameters.update(vo_params)
    
    # Declare launch arguments
    cam_topic_arg = DeclareLaunchArgument(
        'cam_topic',
        default_value='/camera/image_raw',
        description='Camera topic to subscribe to'
    )
    
    # SVO Visual Odometry node
    svo_node = Node(
        package='svo_ros',
        executable='vo',
        name='svo',
        output='screen',
        parameters=[node_parameters],
    )
    
    return LaunchDescription([
        cam_topic_arg,
        svo_node,
    ])


