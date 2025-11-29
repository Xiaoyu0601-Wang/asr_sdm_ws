import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    asr_sdm_vio_ros2_share_dir = get_package_share_directory('asr_sdm_vio_ros2')

    bag_path_arg = DeclareLaunchArgument(
        'bag_path',
        default_value=os.path.expanduser('~/asr_sdm_ws/datasheet/MH_01_easy_ros2'),
        description='ROS2 bag 目录 (支持 ~)'
    )
    left_topic_arg = DeclareLaunchArgument(
        'left_topic', default_value='/cam0/image_raw',
        description='左目图像话题'
    )
    right_topic_arg = DeclareLaunchArgument(
        'right_topic', default_value='/cam1/image_raw',
        description='右目图像话题'
    )
    imu_topic_arg = DeclareLaunchArgument(
        'imu_topic', default_value='/imu0',
        description='IMU话题'
    )
    calib_yaml_arg = DeclareLaunchArgument(
        'calib_yaml',
        default_value=os.path.join(
            asr_sdm_vio_ros2_share_dir, 'param', 'calib', 'euroc_stereo.yaml'),
        description='相机标定（EuRoC 立体）'
    )
    imu_calib_yaml_arg = DeclareLaunchArgument(
        'imu_calib_yaml',
        default_value=os.path.join(
            asr_sdm_vio_ros2_share_dir, 'param', 'calib', 'euroc_imu.yaml'),
        description='IMU parameter file'
    )
    with_rviz_arg = DeclareLaunchArgument(
        'with_rviz', default_value='false',
        description='Whether to start RVIZ2'
    )

    bag_path = LaunchConfiguration('bag_path')
    left_topic = LaunchConfiguration('left_topic')
    right_topic = LaunchConfiguration('right_topic')
    imu_topic = LaunchConfiguration('imu_topic')
    calib_yaml = LaunchConfiguration('calib_yaml')
    imu_calib_yaml = LaunchConfiguration('imu_calib_yaml')
    with_rviz = LaunchConfiguration('with_rviz')

    stereo_node = Node(
        package='asr_sdm_vio_ros2',
        executable='vio_frontend_stereo_ros2',
        name='vio_frontend_stereo_ros2',
        output='screen',
        parameters=[
            imu_calib_yaml,
            {
                'use_sim_time': True,
                'left_topic': left_topic,
                'right_topic': right_topic,
                'imu_topic': imu_topic,
                'calib_yaml': calib_yaml,
            }
        ]
    )

    rviz = Node(
        condition=IfCondition(with_rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen'
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

    bag_player = ExecuteProcess(
        cmd=['ros2', 'bag', 'play', bag_path, '-s', 'mcap', '--clock', '-l', '-r', '0.5', '--topics', '/cam0/image_raw', '/cam1/image_raw', '/imu0'],
        output='screen'
    )

    return LaunchDescription([
        bag_path_arg,
        left_topic_arg,
        right_topic_arg,
        imu_topic_arg,
        calib_yaml_arg,
        imu_calib_yaml_arg,
        with_rviz_arg,
        stereo_node,
        tf_bridge,
        rviz,
        bag_player,
    ])
