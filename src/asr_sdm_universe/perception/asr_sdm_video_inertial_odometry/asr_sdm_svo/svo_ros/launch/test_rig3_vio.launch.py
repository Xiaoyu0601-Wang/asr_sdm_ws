#!/usr/bin/env python3
"""
ROS2 Launch file for SVO VIO (Visual-Inertial Odometry) - Rig3 configuration.

Uses the MSCKF ImageProcessor frontend with IMU pre-integration to provide
a rotation prior for feature tracking. This significantly improves tracking
stability during fast motion compared to the pure visual (vo) node.

Usage:
    ros2 launch svo_ros test_rig3_vio.launch.py

Prerequisites (three terminals):
    Terminal 1: ros2 launch svo_ros test_rig3_vio.launch.py
    Terminal 2: ros2 bag play datasheet/airground_rig_s3_ros2
    Terminal 3: rviz2 -d src/.../svo_ros/rviz_config.rviz
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    svo_ros_dir = get_package_share_directory('svo_ros')
    param_file = os.path.join(svo_ros_dir, 'param', 'vio_rig3_params.yaml')

    # VIO node: uses ImageProcessor with IMU for feature tracking
    # Publishes: /features, /tracking_info, /feature_points, /feature_points_tracked,
    #            /feature_points_new, /debug/mono_features
    vio_node = Node(
        package='svo_ros',
        executable='vio',
        name='svo_vio',
        output='screen',
        parameters=[param_file],
    )

    # Static TF: world -> camera (for rviz visualization)
    # Camera is downward-looking; initial pose is at origin
    world_to_cam_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='world_to_cam_tf',
        arguments=[
            '0', '0', '0',   # x y z
            '0', '0', '0',   # roll pitch yaw
            'world',          # parent frame
            'camera',         # child frame (matches bag /camera/* topics)
        ],
        output='screen'
    )

    return LaunchDescription([
        vio_node,
        world_to_cam_tf,
    ])
