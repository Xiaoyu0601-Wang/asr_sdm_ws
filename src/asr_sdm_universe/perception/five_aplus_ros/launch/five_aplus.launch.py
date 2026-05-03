import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory("five_aplus_ros")
    default_model = os.path.join(package_share, "models", "five_aplus_epoch97.onnx")
    onnxruntime_lib = "/home/cortin/.local/onnxruntime/current/lib"
    ld_library_path = onnxruntime_lib + ":" + os.environ.get("LD_LIBRARY_PATH", "")

    return LaunchDescription(
        [
            SetEnvironmentVariable("LD_LIBRARY_PATH", ld_library_path),
            DeclareLaunchArgument("model_path", default_value=default_model),
            DeclareLaunchArgument("input_topic", default_value="/camera/camera/color/image_raw"),
            DeclareLaunchArgument("output_topic", default_value="/five_aplus/image"),
            DeclareLaunchArgument("num_threads", default_value="0"),
            DeclareLaunchArgument("normalize_output", default_value="true"),
            Node(
                package="five_aplus_ros",
                executable="five_aplus_node",
                name="five_aplus_node",
                output="screen",
                parameters=[
                    {
                        "model_path": LaunchConfiguration("model_path"),
                        "input_topic": LaunchConfiguration("input_topic"),
                        "output_topic": LaunchConfiguration("output_topic"),
                        "num_threads": LaunchConfiguration("num_threads"),
                        "normalize_output": LaunchConfiguration("normalize_output"),
                    }
                ],
            ),
        ]
    )
