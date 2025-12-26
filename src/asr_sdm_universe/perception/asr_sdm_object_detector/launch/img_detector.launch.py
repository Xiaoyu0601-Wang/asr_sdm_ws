from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    package_name = 'asr_sdm_object_detector'

    # 获取 YAML 配置文件路径
    pkg_share = get_package_share_directory(package_name)
    config_file = os.path.join(pkg_share, 'config', 'img_detector.param.yaml')

    # 启动参数：test_mode（true=离线批量测试，false=在线订阅图像）
    test_mode_arg = DeclareLaunchArgument(
        'test_mode',
        default_value='false',   # 建议把 yaml 里 test_mode 也改成 false，当默认在线模式
        description='Run object detector in test (offline) mode if true, '
                    'or online ROS image mode if false.'
    )

    img_detector_node = Node(
        package=package_name,
        executable='img_detector_node',
        name='img_detector_node',
        namespace='perception',
        output='screen',
        emulate_tty=True,
        parameters=[
            config_file,
            # 这里的 test_mode 会覆盖 YAML 中的同名参数
            {'test_mode': LaunchConfiguration('test_mode')},
        ],
        remappings=[
            # 在线模式下生效：输入图像 & 输出检测
            ('~/input/image', '/sensing/camera/front/image_raw'),
            ('~/output/rois', '/perception/object_detection/rois'),
            ('~/debug/exe_time_ms', '/perception/object_detection/exe_time_ms'),
        ],
    )

    return LaunchDescription([
        test_mode_arg,
        img_detector_node,
    ])
