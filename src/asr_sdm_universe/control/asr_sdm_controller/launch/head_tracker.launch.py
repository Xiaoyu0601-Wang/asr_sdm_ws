import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # 获取参数文件的路径
    params_file = os.path.join(
        get_package_share_directory('asr_sdm_controller'),
        'config',
        'head_tracker_params.yaml'
    )

    # 创建控制器节点
    controller_node = Node(
        package='asr_sdm_controller',
        executable='asr_sdm_controller',
        name='asr_sdm_controller',
        output='screen',
        parameters=[params_file],
        remappings=[
            ('~/input/cmd_vel', '/cmd_vel'),
            ('~/output/control_cmd', '/control_cmd'),
        ]
    )

    return LaunchDescription([
        controller_node
    ])

