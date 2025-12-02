from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare('asr_sdm_vio_ros2')

    bag_path_arg = DeclareLaunchArgument('bag_path', default_value='/home/lxy/asr_sdm_ws/datasheet/MH_01_easy_ros2', description='ROS2 bag 目录')
    image_topic_arg = DeclareLaunchArgument('image_topic', default_value='/cam0/image_raw', description='图像话题')
    imu_topic_arg = DeclareLaunchArgument('imu_topic', default_value='/imu0', description='IMU话题')
    calib_yaml_arg = DeclareLaunchArgument(
        'calib_yaml',
        default_value=PathJoinSubstitution([pkg_share, 'param', 'calib', 'euroc_mono.yaml']),
        description='相机标定')
    with_rviz_arg = DeclareLaunchArgument('with_rviz', default_value='true', description='是否启动 RViz2')
    play_rate_arg = DeclareLaunchArgument('play_rate', default_value='0.5', description='bag 播放倍率')

    bag_path = LaunchConfiguration('bag_path')
    image_topic = LaunchConfiguration('image_topic')
    imu_topic = LaunchConfiguration('imu_topic')
    calib_yaml = LaunchConfiguration('calib_yaml')
    with_rviz = LaunchConfiguration('with_rviz')
    play_rate = LaunchConfiguration('play_rate')

    vio_node = Node(
        package='asr_sdm_vio_ros2', executable='vio_frontend_ros2', name='vio_frontend_ros2', output='screen',
        parameters=[{'use_sim_time': True, 'image_topic': image_topic, 'imu_topic': imu_topic, 'calib_yaml': calib_yaml}]
    )

    tf_bridge = Node(
        package='asr_sdm_vio_ros2', executable='rig_pose_to_tf', name='rig_pose_to_tf', output='screen',
        parameters=[{'use_sim_time': True, 'pose_topic': 'Rig', 'parent_frame': 'map', 'child_frame': 'rig'}]
    )

    rviz2 = Node(
        condition=IfCondition(with_rviz), package='rviz2', executable='rviz2', name='rviz2', output='screen')

    bag_player = ExecuteProcess(
        cmd=['ros2', 'bag', 'play', bag_path, '-i', 'mcap', '--clock', '-l', '-r', play_rate, '--topics', image_topic, imu_topic],
        output='screen')

    return LaunchDescription([
        LogInfo(msg='[euroc_global_map_mono.launch.py] EuRoC 单目 + 全局地图配置 (ROS2) — 使用 C++ 前端'),
        bag_path_arg, image_topic_arg, imu_topic_arg, calib_yaml_arg, with_rviz_arg, play_rate_arg,
        vio_node, tf_bridge, rviz2, bag_player,
    ])
