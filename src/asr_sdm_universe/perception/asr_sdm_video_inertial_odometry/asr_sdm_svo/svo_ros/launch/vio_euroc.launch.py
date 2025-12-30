from launch import LaunchDescription
from launch_ros.actions import Node
import os


def generate_launch_description():
    # 参数文件路径
    param_file = os.path.join(
        os.getcwd(),
        'src', 'asr_sdm_universe', 'perception',
        'asr_sdm_video_inertial_odometry',
        'asr_sdm_svo', 'svo_ros', 'param',
        'vio_euroc_params.yaml'
    )

    # VIO 前端节点
    vio_node = Node(
        package='svo_ros',
        executable='vio',
        name='svo_vio',
        output='screen',
        parameters=[
            param_file,
            {
                'use_stereo': False,
                'cam0_topic': '/cam0/image_raw',
                'cam1_topic': '/cam1/image_raw',  # 预留
                'imu_topic': '/imu0',
            }
        ]
    )

    # 静态 TF：world -> cam0
    # 等价于命令行：
    #   ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 world cam0
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='world_to_cam0_tf',
        arguments=[
            '0', '0', '0',    # x y z
            '0', '0', '0',    # roll pitch yaw
            'world',          # parent frame
            'cam0'            # child frame，与 /feature_points header.frame_id 一致
        ],
        output='screen'
    )

    return LaunchDescription([
        vio_node,
        static_tf_node,
    ])