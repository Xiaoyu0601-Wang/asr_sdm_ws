from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os


def generate_launch_description():
    bag_path_arg = DeclareLaunchArgument(
        'bag_path',
        default_value='/home/lxy/asr_sdm_ws/datasheet/MH_01_easy_ros2',
        description='ROS2 bag 目录或文件路径（mcap/sqlite3 目录均可）'
    )
    image_topic_arg = DeclareLaunchArgument(
        'image_topic', default_value='/cam0/image_raw',
        description='图像话题名（EuRoC 单目: /cam0/image_raw）'
    )
    imu_topic_arg = DeclareLaunchArgument(
        'imu_topic', default_value='/imu0',
        description='IMU话题名（EuRoC: /imu0）'
    )
    calib_yaml_arg = DeclareLaunchArgument(
        'calib_yaml',
        default_value=os.path.join(
            '/home/lxy/asr_sdm_ws/install/asr_sdm_vio_ros2/share/asr_sdm_vio_ros2/param/calib',
            'euroc_mono.yaml'),
        description='相机标定文件（默认EuRoC单目）'
    )
    with_rviz_arg = DeclareLaunchArgument(
        'with_rviz', default_value='false',
        description='是否启动 RVIZ2 可视化'
    )

    bag_path = LaunchConfiguration('bag_path')
    image_topic = LaunchConfiguration('image_topic')
    imu_topic = LaunchConfiguration('imu_topic')
    calib_yaml = LaunchConfiguration('calib_yaml')
    with_rviz = LaunchConfiguration('with_rviz')

    vio_node = Node(
        package='asr_sdm_vio_ros2',
        executable='vio_frontend_ros2',
        name='vio_frontend_ros2',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'image_topic': image_topic,
            'imu_topic': imu_topic,
            'calib_yaml': calib_yaml,
        }]
    )

    tf_bridge = Node(
        package='asr_sdm_vio_ros2',
        executable='rig_pose_to_tf',
        name='rig_pose_to_tf',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'pose_topic': 'Rig',
            'parent_frame': 'map',
            'child_frame': 'rig',
        }]
    )

    rviz = Node(
        condition=IfCondition(with_rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen'
    )

    bag_player = ExecuteProcess(
        cmd=['ros2', 'bag', 'play', bag_path, '-s', 'mcap', '--clock', '-l', '-r', '0.5', '--topics', '/cam0/image_raw', '/imu0'],
        output='screen'
    )

    return LaunchDescription([
        bag_path_arg,
        image_topic_arg,
        imu_topic_arg,
        calib_yaml_arg,
        with_rviz_arg,
        vio_node,
        tf_bridge,
        rviz,
        bag_player,
    ])
