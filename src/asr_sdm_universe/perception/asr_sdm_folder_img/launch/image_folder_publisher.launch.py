from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'image_folder',
            default_value=os.path.expanduser('/home/wang/testdata2'),
            description='test img'
        ),
        
        DeclareLaunchArgument(
            'publish_rate',
            default_value='0.5',
            description='发布频率'
        ),
        
        Node(
            package='asr_sdm_folder_publisher',
            executable='image_folder_publisher',
            name='image_folder_publisher',
            output='screen',
            parameters=[{
                'image_folder': LaunchConfiguration('image_folder'),
                'publish_rate': LaunchConfiguration('publish_rate'),
            }]
        )
    ])