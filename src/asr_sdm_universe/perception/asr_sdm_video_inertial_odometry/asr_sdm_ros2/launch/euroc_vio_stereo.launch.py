from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, OpaqueFunction
from launch.substitutions import TextSubstitution
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare('asr_sdm_ros2')

    # Arguments (ported from ROS1 euroc_vio_stereo.launch)
    calib_file_arg = DeclareLaunchArgument(
        'calib_file',
        default_value=PathJoinSubstitution([pkg_share, 'param', 'calib', 'euroc_stereo.yaml']),
        description='相机标定文件（对应 ROS1: calib_file）'
    )
    cam0_topic_arg = DeclareLaunchArgument('cam0_topic', default_value='/cam0/image_raw', description='左目图像话题（对应 ROS1: cam0_topic）')
    cam1_topic_arg = DeclareLaunchArgument('cam1_topic', default_value='/cam1/image_raw', description='右目图像话题（对应 ROS1: cam1_topic）')
    imu_topic_arg = DeclareLaunchArgument('imu_topic', default_value='/imu0', description='IMU话题（对应 ROS1: imu_topic）')

    with_rviz_arg = DeclareLaunchArgument('with_rviz', default_value='true', description='是否启动 RViz2')
    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=PathJoinSubstitution([pkg_share, 'rviz_config_vio.rviz']),
        description='RViz 配置文件（对应 ROS1: rviz_config_vio.rviz）'
    )

    # 可选：ROS2 bag 回放（原 ROS1 launch 不包含，此处保留可用性；默认不播放）
    bag_path_arg = DeclareLaunchArgument('bag_path', default_value='', description='ROS2 bag 路径（可选；空=不播放）')
    play_rate_arg = DeclareLaunchArgument('play_rate', default_value='1.0', description='ros2 bag 播放倍率')
    loop_arg = DeclareLaunchArgument('loop', default_value='false', description='循环播放 bag')
    storage_arg = DeclareLaunchArgument('storage_id', default_value='', description='ros2 bag 存储后端（空=自动；rosbag_v2/sqlite3/mcap）')

    # LaunchConfigurations
    bag_path = LaunchConfiguration('bag_path')
    cam0_topic = LaunchConfiguration('cam0_topic')
    cam1_topic = LaunchConfiguration('cam1_topic')
    imu_topic = LaunchConfiguration('imu_topic')
    calib_file = LaunchConfiguration('calib_file')
    with_rviz = LaunchConfiguration('with_rviz')
    rviz_config = LaunchConfiguration('rviz_config')
    play_rate = LaunchConfiguration('play_rate')
    loop = LaunchConfiguration('loop')
    storage_id = LaunchConfiguration('storage_id')

    # SVO node (matches ROS1 euroc_vio_stereo.launch)
    svo_node = Node(
        package='asr_sdm_ros2',
        executable='svo_node',
        name='svo',
        output='screen',
        arguments=['--v=0'],
        parameters=[
            {
                'cam0_topic': cam0_topic,
                'cam1_topic': cam1_topic,
                'imu_topic': imu_topic,
                'calib_file': calib_file,
                'runlc': True,
            },
            PathJoinSubstitution([pkg_share, 'param', 'vio_stereo.yaml']),
        ],
    )

    # Pose -> TF bridge
    tf_bridge = Node(
        package='asr_sdm_ros2',
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

