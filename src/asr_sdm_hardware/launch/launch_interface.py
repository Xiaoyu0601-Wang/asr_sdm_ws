import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # load periphery
    periphery_config_yaml = os.path.join(
        get_package_share_directory('asr_sdm_hardware'),
        'config',
        'periphery_config.yaml')

    with open(periphery_config_yaml, 'r') as ymlfile:
        periphery_config = yaml.safe_load(ymlfile)
    
    return LaunchDescription([
        Node(
            package='asr_sdm_hardware',
            executable='asr_sdm_hardware_node',
            name='asr_sdm_hardware',
            remappings=[
                ('/input/can_frame', '/robot/cmd_vel'),
                ('/sensor_data', '/lidar/data')
            ],
            output='screen',
            parameters=[periphery_config]
        ),
    ])