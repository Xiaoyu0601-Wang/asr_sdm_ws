import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # load periphery
    periphery_config = os.path.join(
        get_package_share_directory('asr_sdm_individual_parameters'),
        'config',
        'asr_sdm_10_01.yaml')

    return LaunchDescription([
        Node(
            package='asr_sdm_hardware',
            executable='asr_sdm_hardware_node',
            name='asr_sdm_hardware',
            # remappings=[
            #     ('/input/can_frame', '/robot/cmd_vel'),
            #     ('/sensor_data', '/lidar/data')
            # ],
            output='screen',
            parameters=[periphery_config,
                        # {'use_sim_time': True},
                        # {'uart_can.uart_port': '/dev/ttyS3'},
                        # {'uart_can.uart_baudrate': 115200},
                        # {'imu_wheeltec_n100.uart_port': '/dev/ttyS4'},
                        # {'imu_wheeltec_n100.uart_baudrate': 115200},
                        {'topic_asr_sdm_cmd': '~/asr_sdm_cmd'}]
        ),
    ])