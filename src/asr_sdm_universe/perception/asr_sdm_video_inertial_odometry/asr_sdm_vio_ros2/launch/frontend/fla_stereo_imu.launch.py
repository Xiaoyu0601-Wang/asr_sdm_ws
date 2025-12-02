from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare('asr_sdm_vio_ros2')

    bag_path_arg = DeclareLaunchArgument('bag_path', default_value='', description='(可选) ROS2 bag 路径')
    cam0_topic_arg = DeclareLaunchArgument('cam0_topic', default_value='/sync/cam0/image_raw', description='左目图像话题')
    cam1_topic_arg = DeclareLaunchArgument('cam1_topic', default_value='/sync/cam1/image_raw', description='右目图像话题')
    imu_topic_arg = DeclareLaunchArgument('imu_topic', default_value='/sync/imu/imu', description='IMU 话题')
    calib_yaml_arg = DeclareLaunchArgument(
        'calib_yaml',
        default_value=PathJoinSubstitution([pkg_share, 'share', 'asr_sdm_vio_ros2', 'param', 'calib', 'fla_stereo_imu.yaml']),
        description='双目相机标定（FLA rig）'
    )
    with_rviz_arg = DeclareLaunchArgument('with_rviz', default_value='false', description='是否启动 RViz2')

    bag_path = LaunchConfiguration('bag_path')
    cam0_topic = LaunchConfiguration('cam0_topic')
    cam1_topic = LaunchConfiguration('cam1_topic')
    imu_topic = LaunchConfiguration('imu_topic')
    calib_yaml = LaunchConfiguration('calib_yaml')
    with_rviz = LaunchConfiguration('with_rviz')

    vio_node = Node(
        package='asr_sdm_vio_ros2', executable='vio_frontend_stereo_ros2', name='vio_frontend_stereo_ros2', output='screen',
        parameters=[{'use_sim_time': True, 'left_topic': cam0_topic, 'right_topic': cam1_topic, 'imu_topic': imu_topic, 'calib_yaml': calib_yaml}]
    )

    tf_bridge = Node(
        package='asr_sdm_vio_ros2', executable='rig_pose_to_tf', name='rig_pose_to_tf', output='screen',
        parameters=[{'use_sim_time': True, 'pose_topic': 'Rig', 'parent_frame': 'map', 'child_frame': 'rig'}]
    )

    rviz2 = Node(
        condition=IfCondition(with_rviz), package='rviz2', executable='rviz2', name='rviz2', output='screen')

    bag_player = ExecuteProcess(
        condition=IfCondition(bag_path),
        cmd=['ros2', 'bag', 'play', bag_path, '-i', 'mcap', '--clock', '-l', '-r', '0.5', '--topics', cam0_topic, cam1_topic, imu_topic],
        output='screen')

    return LaunchDescription([
        bag_path_arg, cam0_topic_arg, cam1_topic_arg, imu_topic_arg, calib_yaml_arg, with_rviz_arg,
        LogInfo(msg='[fla_stereo_imu.launch.py] FLA Stereo IMU 前端 + (可选) bag 播放'),
        vio_node, tf_bridge, rviz2, bag_player,
    ])
