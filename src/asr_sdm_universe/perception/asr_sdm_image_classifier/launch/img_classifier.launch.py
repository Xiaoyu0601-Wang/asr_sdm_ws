from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    package_name = 'asr_sdm_image_classifier'

    # 获取 YAML 配置文件路径
    pkg_share = get_package_share_directory(package_name)
    config_file = os.path.join(pkg_share, 'config', 'img_classifier.param.yaml')

    # 启动参数：test_mode（true=离线批量测试，false=在线订阅图像）
    test_mode_arg = DeclareLaunchArgument(
        'test_mode',
        default_value='false',   # 建议 YAML 中 test_mode 也设为 false
        description='Run image classifier in test (offline) mode if true, '
                    'or online ROS image mode if false.'
    )

    img_classifier_node = Node(
        package=package_name,
        executable='img_classifier_node',
        name='img_classifier_node',
        namespace='perception',
        output='screen',
        emulate_tty=True,
        parameters=[
            config_file,
            {'test_mode': LaunchConfiguration('test_mode')},
        ],
        remappings=[
            # 在线模式下：从detector订阅图像
            ('~/input/rois', '/perception/object_detection/rois'),
            # 输出带标签的调试图像
            ('~/output/image', '/perception/image_classification/image'),
            # 输出分类结果字符串
            ('~/output/label', '/perception/image_classification/label'),
        ],
    )

    return LaunchDescription([
        test_mode_arg,
        img_classifier_node,
    ])
