"""Turn-key launch file for the Pinocchio visualization stack."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    use_rviz = LaunchConfiguration("use_rviz")
    use_pinocchio = LaunchConfiguration("use_pinocchio")

    urdf_path = os.path.join(
        get_package_share_directory("asr_sdm_description"),
        "urdf",
        "underwater_snakerobot.urdf",
    )

    # Robot State Publisher still expects the XML string, so keep a copy.
    with open(urdf_path, "r") as f:
        robot_description = f.read()

    rviz_config = os.path.join(
        get_package_share_directory("asr_sdm_launch"),
        "config",
        "asr_sdm_pinocchio.rviz",
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description}],
    )

    joint_state_publisher = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        output="screen",
    )

    pinocchio_dynamics_node = Node(
        package="asr_sdm_kinematic_dynamic_model",
        executable="pinocchio_dynamics_node",
        name="pinocchio_dynamics_node",
        output="screen",
        parameters=[
            {
                "robot_description_path": urdf_path,
                "use_free_flyer": True,
                "publish_period_ms": 200,
            }
        ],
        condition=None,
    )

    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_rviz",
                default_value="true",
                description="Start RViz2 for visualization",
            ),
            DeclareLaunchArgument(
                "use_pinocchio",
                default_value="true",
                description="Start pinocchio_dynamics_node",
            ),
            robot_state_publisher,
            joint_state_publisher,
            pinocchio_dynamics_node,
            rviz2,
        ]
    )

