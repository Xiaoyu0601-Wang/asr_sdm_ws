from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare('asr_sdm_ros2')

    cam_name_arg = DeclareLaunchArgument('cam_name', default_value='euroc_mono', description='相机标定名（对应 param/calib/<cam_name>.yaml）')
    bag_path_arg = DeclareLaunchArgument('bag_path', default_value='', description='(可选) ROS2 bag 路径')
    image_topic_arg = DeclareLaunchArgument('image_topic', default_value='camera/image_raw', description='图像话题')
    map_scale_arg = DeclareLaunchArgument('map_scale', default_value='1.0', description='地图尺度（兼容旧参数，不在前端使用）')

    cam_name = LaunchConfiguration('cam_name')
    bag_path = LaunchConfiguration('bag_path')
    image_topic = LaunchConfiguration('image_topic')
    map_scale = LaunchConfiguration('map_scale')

    calib_yaml = PathJoinSubstitution([pkg_share, 'share', 'asr_sdm_ros2', 'param', 'calib', cam_name, TextSubstitution(text='.yaml')])

    vio_node = Node(
        package='asr_sdm_ros2', executable='vio_frontend_ros2', name='vio_frontend_ros2', output='screen',
        parameters=[{'use_sim_time': True, 'image_topic': image_topic, 'imu_topic': '/imu0', 'calib_yaml': calib_yaml, 'map_scale': map_scale}]
    )

    tf_bridge = Node(
        package='asr_sdm_ros2', executable='rig_pose_to_tf', name='rig_pose_to_tf', output='screen',
        parameters=[{'use_sim_time': True, 'pose_topic': 'Rig', 'parent_frame': 'map', 'child_frame': 'rig'}]
    )

    rviz2 = Node(
        package='rviz2', executable='rviz2', name='rviz2', output='screen'
    )

    bag_player = ExecuteProcess(
        condition=IfCondition(bag_path),
        cmd=['ros2', 'bag', 'play', bag_path, '-i', 'mcap', '--clock', '-l', '-r', '0.5', '--topics', image_topic, '/imu0'],
        output='screen'
    )

    return LaunchDescription([
        LogInfo(msg='[run_from_bag.launch.py] 单目前端(ROS2) + (可选) bag 播放'),
        cam_name_arg, bag_path_arg, image_topic_arg, map_scale_arg,
        vio_node, tf_bridge, rviz2, bag_player,
    ])
