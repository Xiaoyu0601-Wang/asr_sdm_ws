import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    interface_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('your_package'),
                'launch',
                'interface_launch.py'
            )
        )
    )

    return LaunchDescription([
        interface_launch,
        # joystick driver node
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
            parameters=[{
                "dev": "/dev/input/js0",   # joystick device file
                "deadzone": 0.1,
                "autorepeat_rate": 20.0
            }]
        ),

        # custom joystick to cmd_vel node
        Node(
            package='joystick_teleop_cpp',   # your package name
            executable='joystick_teleop',    # C++ executable file name
            name='joystick_teleop',
            output='screen',
            parameters=[{
                "linear_axis": 1,
                "angular_axis": 0,
                "linear_scale": 0.5,
                "angular_scale": 1.0
            }]
        )
    ])