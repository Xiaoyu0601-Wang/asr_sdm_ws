import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Use os.path.expanduser to handle '~'
    default_bag_path = os.path.expanduser('~/asr_sdm_ws/datasheet/MH_01_easy_ros2')

    bag_path_arg = DeclareLaunchArgument(
        'bag_path',
        default_value=default_bag_path,
        description='ROS2 bag directory path (supports ~)'
    )
    image_topic_arg = DeclareLaunchArgument(
        'image_topic', default_value='/cam0/image_raw',
        description='Image topic name'
    )
    imu_topic_arg = DeclareLaunchArgument(
        'imu_topic', default_value='/imu0',
        description='IMU topic name'
    )

    bag_path = LaunchConfiguration('bag_path')
    image_topic = LaunchConfiguration('image_topic')
    imu_topic = LaunchConfiguration('imu_topic')

    minimal_node = Node(
        package='asr_sdm_vio_ros2',
        executable='minimal_vio_frontend',
        name='minimal_vio_frontend',
        output='screen',
        parameters=[{
            'image_topic': image_topic,
            'imu_topic': imu_topic,
            'report_period': 1.0,
            'use_sim_time': True,  # Necessary for bag playback
        }]
    )

    bag_player = ExecuteProcess(
        # Add --clock to publish simulation time
        cmd=['ros2', 'bag', 'play', bag_path, '--clock', '-l', '-r', '0.5'],
        output='screen'
    )

    return LaunchDescription([
        bag_path_arg,
        image_topic_arg,
        imu_topic_arg,
        minimal_node,
        bag_player,
    ])
