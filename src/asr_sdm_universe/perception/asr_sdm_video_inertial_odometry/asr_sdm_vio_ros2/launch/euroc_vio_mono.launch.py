from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare('asr_sdm_vio_ros2')

    # Arguments
    bag_path_arg = DeclareLaunchArgument(
        'bag_path',
        default_value='/home/lxy/asr_sdm_ws/datasheet/MH_01_easy_ros2',
        description='ROS2 bag 目录（mcap/sqlite3 均可）'
    )
    image_topic_arg = DeclareLaunchArgument(
        'image_topic', default_value='/cam0/image_raw',
        description='单目图像话题（EuRoC: /cam0/image_raw）'
    )
    imu_topic_arg = DeclareLaunchArgument(
        'imu_topic', default_value='/imu0',
        description='IMU 话题（EuRoC: /imu0）'
    )
    calib_yaml_arg = DeclareLaunchArgument(
        'calib_yaml',
        default_value=PathJoinSubstitution([
            pkg_share, 'param', 'calib', 'euroc_mono.yaml'
        ]),
        description='单目相机标定文件'
    )
    with_rviz_arg = DeclareLaunchArgument(
        'with_rviz', default_value='true',
        description='是否启动 RViz2'
    )
    play_rate_arg = DeclareLaunchArgument(
        'play_rate', default_value='1.0',
        description='ros2 bag 播放倍率'
    )
    loop_arg = DeclareLaunchArgument(
        'loop', default_value='false',
        description='循环播放 bag'
    )
    storage_arg = DeclareLaunchArgument(
        'storage_id', default_value='',
        description='ros2 bag 存储后端（空=自动；可填 rosbag_v2/sqlite3/mcap）'
    )

    bag_path = LaunchConfiguration('bag_path')
    image_topic = LaunchConfiguration('image_topic')
    imu_topic = LaunchConfiguration('imu_topic')
    calib_yaml = LaunchConfiguration('calib_yaml')
    with_rviz = LaunchConfiguration('with_rviz')
    play_rate = LaunchConfiguration('play_rate')
    loop = LaunchConfiguration('loop')
    storage_id = LaunchConfiguration('storage_id')

    # VIO mono frontend (C++)
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

    # Pose -> TF bridge
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

    # RViz2
    rviz2 = Node(
        condition=IfCondition(with_rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', PathJoinSubstitution([pkg_share, 'rviz_config_vio_enhanced.rviz'])],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    # ros2 bag play（根据参数动态构造，支持 rosbag_v2/sqlite3/mcap；bag_path 为空则不播放）
    def make_bag_player(context, *args, **kwargs):
        path = context.perform_substitution(bag_path)
        if path is None or path == '' or path.lower() == 'none':
            return []
        rate = context.perform_substitution(play_rate) or '1.0'
        loop_flag = context.perform_substitution(loop).lower()
        storage = context.perform_substitution(storage_id)
        img = context.perform_substitution(image_topic)
        imu = context.perform_substitution(imu_topic)

        cmd = ['ros2', 'bag', 'play', path, '--clock', '-r', rate, '--topics', img, imu]
        # 自动识别 rosbag1
        if path.endswith('.bag') and not storage:
            storage = 'rosbag_v2'
        if storage:
            cmd += ['-s', storage]
        if loop_flag in ['1', 'true', 'True']:
            cmd.append('-l')
        return [ExecuteProcess(cmd=cmd, output='screen')]

    bag_player = OpaqueFunction(function=make_bag_player)

    return LaunchDescription([
        bag_path_arg,
        image_topic_arg,
        imu_topic_arg,
        calib_yaml_arg,
        with_rviz_arg,
        play_rate_arg,
        loop_arg,
        storage_arg,
        LogInfo(msg='[euroc_vio_mono.launch.py] 启动 EuRoC 单目 VIO (ROS2)'),
        vio_node,
        tf_bridge,
        rviz2,
        bag_player,
    ])

