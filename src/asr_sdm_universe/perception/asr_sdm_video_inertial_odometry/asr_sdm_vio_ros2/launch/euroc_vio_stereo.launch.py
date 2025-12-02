from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare('asr_sdm_vio_ros2')

    # Arguments (porting from ROS1 euroc_vio_stereo.launch)
    bag_path_arg = DeclareLaunchArgument(
        'bag_path',
        default_value='/home/lxy/asr_sdm_ws/datasheet/MH_01_easy_ros2',
        description='ROS2 bag 目录（mcap/sqlite3 均可）或 ROS1 .bag（自动识别）'
    )
    left_topic_arg = DeclareLaunchArgument('left_topic', default_value='/cam0/image_raw', description='左目图像话题')
    right_topic_arg = DeclareLaunchArgument('right_topic', default_value='/cam1/image_raw', description='右目图像话题')
    imu_topic_arg = DeclareLaunchArgument('imu_topic', default_value='/imu0', description='IMU 话题')

    calib_yaml_arg = DeclareLaunchArgument(
        'calib_yaml',
        default_value=PathJoinSubstitution([pkg_share, 'param', 'calib', 'euroc_stereo.yaml']),
        description='双目相机标定文件 (原 ROS1 calib_file)'
    )
    runlc_arg = DeclareLaunchArgument('runlc', default_value='true', description='是否启用回环（与参数文件一致）')

    # 可选：ROS2 参数文件（已将原 ROS1 vio_stereo.yaml 转为 ROS2 格式）
    ros2_param_file_arg = DeclareLaunchArgument(
        'ros2_param_file',
        default_value=PathJoinSubstitution([pkg_share, 'param', 'vio_stereo.ros2.yaml']),
        description='ROS2 参数文件（可选）'
    )

    with_rviz_arg = DeclareLaunchArgument('with_rviz', default_value='true', description='是否启动 RViz2')
    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=PathJoinSubstitution([pkg_share, 'rviz_config_vio_enhanced.rviz']),
        description='RViz 配置文件'
    )

    play_rate_arg = DeclareLaunchArgument('play_rate', default_value='1.0', description='ros2 bag 播放倍率')
    loop_arg = DeclareLaunchArgument('loop', default_value='false', description='循环播放 bag')
    storage_arg = DeclareLaunchArgument('storage_id', default_value='', description='ros2 bag 存储后端（空=自动；rosbag_v2/sqlite3/mcap）')

    enable_vo_arg = DeclareLaunchArgument('enable_vo', default_value='true', description='是否启用前端 VO（便于排障可关闭）')
    passthrough_arg = DeclareLaunchArgument('publish_passthrough', default_value='true', description='转发左目图像到 /image_with_features')

    parent_frame_arg = DeclareLaunchArgument('parent_frame', default_value='map', description='TF 父坐标系')
    child_frame_arg = DeclareLaunchArgument('child_frame', default_value='rig', description='TF 子坐标系')

    # LaunchConfigurations
    bag_path = LaunchConfiguration('bag_path')
    left_topic = LaunchConfiguration('left_topic')
    right_topic = LaunchConfiguration('right_topic')
    imu_topic = LaunchConfiguration('imu_topic')
    calib_yaml = LaunchConfiguration('calib_yaml')
    runlc = LaunchConfiguration('runlc')
    with_rviz = LaunchConfiguration('with_rviz')
    rviz_config = LaunchConfiguration('rviz_config')
    play_rate = LaunchConfiguration('play_rate')
    loop = LaunchConfiguration('loop')
    storage_id = LaunchConfiguration('storage_id')
    enable_vo = LaunchConfiguration('enable_vo')
    publish_passthrough = LaunchConfiguration('publish_passthrough')
    parent_frame = LaunchConfiguration('parent_frame')
    child_frame = LaunchConfiguration('child_frame')
    ros2_param_file = LaunchConfiguration('ros2_param_file')

    # Stereo VIO frontend (C++) — 参数合并：param_file + 显式参数
    vio_node = Node(
        package='asr_sdm_vio_ros2',
        executable='vio_frontend_stereo_ros2',
        name='vio_frontend_stereo_ros2',
        output='screen',
        parameters=[
            ros2_param_file,
            {
                'use_sim_time': True,
                'left_topic': left_topic,
                'right_topic': right_topic,
                'imu_topic': imu_topic,
                'calib_yaml': calib_yaml,
                'runlc': runlc,
                'enable_vo': enable_vo,
                'publish_passthrough': publish_passthrough,
            }
        ]
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
            'parent_frame': parent_frame,
            'child_frame': child_frame,
        }]
    )

    # RViz2
    rviz2 = Node(
        condition=IfCondition(with_rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    # ros2 bag play（根据参数动态构造，支持 rosbag_v2/sqlite3/mcap；bag_path 为空则不播放）
    def make_bag_player(context, *args, **kwargs):
        path = context.perform_substitution(bag_path)
        if path is None or path == '' or path.lower() == 'none':
            return []
        rate = context.perform_substitution(play_rate) or '1.0'
        storage = context.perform_substitution(storage_id)
        loop_flag = context.perform_substitution(loop).lower()
        left = context.perform_substitution(left_topic)
        right = context.perform_substitution(right_topic)
        imu = context.perform_substitution(imu_topic)
        cmd = ['ros2', 'bag', 'play', path, '--clock', '-r', rate, '--topics', left, right, imu]
        if path.endswith('.bag') and not storage:
            storage = 'rosbag_v2'
        if storage:
            cmd += ['-s', storage]
        if loop_flag in ['1', 'true', 'True']:
            cmd.append('-l')
        return [ExecuteProcess(cmd=cmd, output='screen')]

    bag_player = OpaqueFunction(function=make_bag_player)

    return LaunchDescription([
        # Ported/extended arguments
        bag_path_arg,
        left_topic_arg,
        right_topic_arg,
        imu_topic_arg,
        calib_yaml_arg,
        ros2_param_file_arg,
        runlc_arg,
        with_rviz_arg,
        rviz_config_arg,
        play_rate_arg,
        loop_arg,
        storage_arg,
        enable_vo_arg,
        passthrough_arg,
        parent_frame_arg,
        child_frame_arg,

        LogInfo(msg='[euroc_vio_stereo.launch.py] 启动 EuRoC 双目 VIO (ROS2) + 参数文件'),
        vio_node,
        tf_bridge,
        rviz2,
        bag_player,
    ])

