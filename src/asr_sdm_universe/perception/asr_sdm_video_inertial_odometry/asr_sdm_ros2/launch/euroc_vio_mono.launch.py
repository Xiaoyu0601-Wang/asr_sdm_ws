from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare('asr_sdm_ros2')

    # Arguments (ported from ROS1 euroc_vio_mono.launch)
    calib_file_arg = DeclareLaunchArgument(
        'calib_file',
        default_value=PathJoinSubstitution([pkg_share, 'param', 'calib', 'euroc_mono.yaml']),
        description='相机标定文件（对应 ROS1: calib_file）'
    )
    cam0_topic_arg = DeclareLaunchArgument(
        'cam0_topic',
        default_value='/cam0/image_raw',
        description='图像话题（对应 ROS1: cam0_topic）'
    )
    cam1_topic_arg = DeclareLaunchArgument(
        'cam1_topic',
        default_value='/cam1/image_raw',
        description='（保留与 ROS1 一致）第二路图像话题（对应 ROS1: cam1_topic）'
    )
    imu_topic_arg = DeclareLaunchArgument(
        'imu_topic',
        default_value='/imu0',
        description='IMU 话题（对应 ROS1: imu_topic）'
    )
    runlc_arg = DeclareLaunchArgument(
        'runlc',
        default_value='true',
        description='是否启用回环（对应 ROS1: runlc）'
    )

    with_rviz_arg = DeclareLaunchArgument(
        'with_rviz',
        default_value='true',
        description='是否启动 RViz2'
    )
    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=PathJoinSubstitution([pkg_share, 'rviz_config_vio.rviz']),
        description='RViz 配置文件（对应 ROS1: rviz_config_vio.rviz）'
    )

    calib_file = LaunchConfiguration('calib_file')
    cam0_topic = LaunchConfiguration('cam0_topic')
    cam1_topic = LaunchConfiguration('cam1_topic')
    imu_topic = LaunchConfiguration('imu_topic')
    runlc = LaunchConfiguration('runlc')
    with_rviz = LaunchConfiguration('with_rviz')
    rviz_config = LaunchConfiguration('rviz_config')

    # SVO node (matches ROS1 euroc_vio_mono.launch)
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
                'runlc': runlc,
                'use_sim_time': True,
            },
            PathJoinSubstitution([pkg_share, 'param', 'vio_mono.yaml']),
        ],
    )

    rviz2 = Node(
        condition=IfCondition(with_rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    return LaunchDescription([
        SetEnvironmentVariable('ASAN_OPTIONS', 'new_delete_type_mismatch=0:detect_leaks=0:allocator_may_return_null=1'),
        calib_file_arg,
        cam0_topic_arg,
        cam1_topic_arg,
        imu_topic_arg,
        runlc_arg,
        with_rviz_arg,
        rviz_config_arg,
        LogInfo(msg='[euroc_vio_mono.launch.py] 启动 EuRoC 单目 VIO（ROS1 行为复刻：svo_node + vio_mono.yaml + RViz）'),
        svo_node,
        rviz2,
    ])
