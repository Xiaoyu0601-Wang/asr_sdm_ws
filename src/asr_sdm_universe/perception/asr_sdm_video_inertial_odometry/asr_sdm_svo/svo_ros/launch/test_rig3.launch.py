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
from launch.conditions import IfCondition


def generate_launch_description():
    # Get the package share directory
    svo_ros_dir = get_package_share_directory('svo_ros')

    camera_yaml_path = os.path.join(svo_ros_dir, 'param', 'camera_pinhole.yaml')
    vo_yaml_path = os.path.join(svo_ros_dir, 'param', 'vo_rig3_stable.yaml')

    # Declare launch arguments
    cam_topic_arg = DeclareLaunchArgument(
        'cam_topic',
        default_value='/camera/image_raw',
        description='Camera topic to subscribe to'
    )

    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='false',
        description='Launch RViz2 for visualization'
    )

    fast_type_arg = DeclareLaunchArgument(
        'fast_type',
        default_value='12',
        description='FAST detector type: 7 / 8 / 9 / 10 / 11 / 12'
    )

    max_queue_size_arg = DeclareLaunchArgument(
        'max_queue_size',
        default_value='1',
        description='Max image queue size for processing thread'
    )

    drop_frames_arg = DeclareLaunchArgument(
        'drop_frames',
        default_value='true',
        description='Drop oldest frames when queue is full'
    )

    enable_frame_throttle_arg = DeclareLaunchArgument(
        'enable_frame_throttle',
        default_value='false',
        description='Enable input frame throttling before queueing'
    )

    target_fps_arg = DeclareLaunchArgument(
        'target_fps',
        default_value='15.0',
        description='Target accepted input FPS when throttling is enabled'
    )


    svo_node = Node(
        package='svo_ros',
        executable='vo',
        name='asr_sdm_svo',
        output='screen',
        parameters=[
            camera_yaml_path,
            vo_yaml_path,
            {
                # Allow overriding via launch arg
                'cam_topic': LaunchConfiguration('cam_topic'),

                # Initial camera orientation to point downward
                'init_rx': 3.14,
                'init_ry': 0.0,
                'init_rz': 0.0,
                'fast_type': LaunchConfiguration('fast_type'),
                'max_queue_size': LaunchConfiguration('max_queue_size'),
                'drop_frames': LaunchConfiguration('drop_frames'),
                'enable_frame_throttle': LaunchConfiguration('enable_frame_throttle'),
                'target_fps': LaunchConfiguration('target_fps'),
                'use_async_processing': False,
                'enable_visualization': False,
                'publish_markers': False,
                'publish_dense_input': False,
                'publish_every_nth_img': 2,
                'publish_every_nth_dense_input': 4,
                'publish_map_every_frame': False,
            },
        ],
    )

    # RViz2 launch configuration
    rviz_config_path = os.path.join(svo_ros_dir, 'rviz_config.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        condition=IfCondition(LaunchConfiguration('rviz')),
        output='screen'
    )

    return LaunchDescription([
        cam_topic_arg,
        rviz_arg,
        fast_type_arg,
        max_queue_size_arg,
        drop_frames_arg,
        enable_frame_throttle_arg,
        target_fps_arg,
        svo_node,
        rviz_node,
    ])
