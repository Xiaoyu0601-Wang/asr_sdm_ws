from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # 参数文件路径（使用 ament_index，避免依赖当前工作目录）
    pkg_share = get_package_share_directory('svo_ros')
    param_file = os.path.join(pkg_share, 'param', 'vio_euroc_params.yaml')

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

    # 为后端节点指定独立的参数文件
    backend_param_file = os.path.join(pkg_share, 'param', 'msckf_backend_params.yaml')

    # MSCKF 后端节点
    msckf_backend_node = Node(
        package='svo_ros',
        executable='msckf_backend_node',
        name='msckf_backend',
        output='screen',
        parameters=[
            # 加载 vio_euroc_params.yaml 以获取共享的 T_cam_imu 等
            param_file,
            # 加载后端专属参数，这将覆盖 param_file 中的同名参数
            backend_param_file,
            {
                # 输入 topic（与前端对齐）
                'imu_topic': '/imu0',
                # 前端发布 CameraMeasurement 在 /features
                'feature_topic': '/features',

                # 输出 frame
                'fixed_frame_id': 'world',
                'child_frame_id': 'robot',
                'publish_tf': True,

                # Feature triangulation sanity checks (backend)
                'feature.config.min_depth': 0.1,
                'feature.config.max_depth': 150.0,
                'feature.config.max_rms_reproj_error': 0.2,
            }
        ]
    )

    # 静态 TF：world -> cam0
    # 注意：这里不要重复启动 static_transform_publisher。
    # 如果需要该静态 TF，请只在一个地方启动（例如仅在此 launch 中）。
    # 当前为了避免 /world_to_cam0_tf 重名节点告警，这里先不启动。

    return LaunchDescription([
        vio_node,
        msckf_backend_node,
    ])
