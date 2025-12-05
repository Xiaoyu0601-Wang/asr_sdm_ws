from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    video_node = Node(
        package='asr_sdm_video_enhancement',
        executable='asr_sdm_video_enhancement',  # ros2 pkg executables 里看到的名字
        name='asr_sdm_video_enhancement',
        output='screen',
        parameters=[
            {'airlight': 255},
            {'scale': 0.5},
        ],
        remappings=[
            # 这里用的是你 C++ 里写的 "~/input/image" / "~/output/image"
            ('~/input/image',  '/camera/camera/color/image_rect_raw'),
            ('~/output/image', '/asr_sdm_video_enhancement/output/image'),
        ],
    )

    return LaunchDescription([
        video_node,
    ])
