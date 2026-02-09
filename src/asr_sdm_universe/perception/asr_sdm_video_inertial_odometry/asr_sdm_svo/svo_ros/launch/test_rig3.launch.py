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

    camera_yaml_path = os.path.join(svo_ros_dir, 'param', 'realsense_camera_atan.yaml')
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

    # SVO Visual Odometry node
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
        svo_node,
        rviz_node,
    ])
