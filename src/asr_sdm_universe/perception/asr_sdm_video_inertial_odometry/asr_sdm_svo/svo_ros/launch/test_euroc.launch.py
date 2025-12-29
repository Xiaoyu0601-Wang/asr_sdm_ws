#!/usr/bin/env python3
"""ROS2 Launch file for SVO Visual Odometry - EuRoC Dataset configuration.

本 launch 的主要目标：
1) 正确加载相机参数 + VO 参数，并传给 `svo_ros/vo` 节点。
2) 解决我们在调试过程中遇到的“参数文件格式不一致导致参数不生效”的问题。

重要背景：
- 该工程里曾出现两种 VO 参数 yaml 格式：
  A) ROS2 nested 格式：
     svo:
       ros__parameters:
         grid_size: ...
  B) flat 平铺格式：
     grid_size: ...
- `svo_ros/vo` 节点内部通过 `vk::getParam(node, "key", default)` 读取参数，
  因此最终传入 Node 的 parameters 必须是“扁平 key/value”。

因此本文件做了：
- 自动兼容 nested/flat 两种格式
- 在 launch 时打印关键参数，避免“以为改了但实际没生效”
"""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node


def _load_yaml(path: str):
    """读取 yaml 文件。

    注意：yaml.safe_load 可能返回 None（空文件），这里统一转成 {}。
    """
    with open(path, 'r') as f:
        data = yaml.safe_load(f)
    return {} if data is None else data


def generate_launch_description():
    # 获取 svo_ros 包的 share 目录（注意：launch 实际读取的是 install/share 下的文件）。
    # 这也是为什么你改了 src 下的 yaml 后，需要 colcon build + source install 才能生效。
    svo_ros_dir = get_package_share_directory('svo_ros')

    camera_yaml_path = os.path.join(svo_ros_dir, 'param', 'camera_euroc.yaml')
    vo_yaml_path = os.path.join(svo_ros_dir, 'param', 'vo_euroc_stable.yaml')

    camera_params = _load_yaml(camera_yaml_path)
    vo_params_raw = _load_yaml(vo_yaml_path)

    # 兼容两种 VO yaml 格式：
    #  1) ROS2 nested: {svo: {ros__parameters: {...}}}
    #  2) flat: {grid_size:..., max_fts:..., ...}
    #
    # 最终我们要得到一个“扁平字典” vo_params，直接 update 到 node_parameters。
    if isinstance(vo_params_raw, dict) and 'svo' in vo_params_raw and isinstance(vo_params_raw['svo'], dict):
        vo_params = vo_params_raw.get('svo', {}).get('ros__parameters', {})
        if not vo_params:
            # 兜底：有些文件可能写成 {svo: {...}} 但没有 ros__parameters 这一层
            vo_params = vo_params_raw.get('svo', {})
    else:
        vo_params = vo_params_raw

    # 基础启动参数
    node_parameters = {
        # 订阅 EuRoC 左目图像
        'cam_topic': '/cam0/image_raw',
        # rosbag 播放使用 --clock 时必须启用仿真时间，否则时间戳会不匹配
        'use_sim_time': True,
        # 初始姿态（RPY），一般 EuRoC 可保持 0
        'init_rx': 0.0,
        'init_ry': 0.0,
        'init_rz': 0.0,
    }

    # 合并参数（VO 参数如果与相机参数冲突，以 VO 为准）
    node_parameters.update(camera_params)
    node_parameters.update(vo_params)

    # Launch 阶段打印关键参数：
    # 用途：快速确认“install/share 下的 yaml 是否更新”“参数是否真的传进 Node”。
    print('[test_euroc.launch.py] camera_yaml:', camera_yaml_path)
    print('[test_euroc.launch.py] vo_yaml:', vo_yaml_path)
    for k in [
        'grid_size', 'max_fts', 'n_pyr_levels', 'triang_min_corner_score',
        'quality_min_fts', 'quality_max_drop_fts', 'reproj_thresh', 'poseoptim_thresh',
        'kfselect_mindist', 'max_n_kfs', 'map_scale', 'patch_match_thresh_factor',
        'klt_min_level', 'klt_max_level', 'subpix_n_iter',
    ]:
        if k in node_parameters:
            print(f"[test_euroc.launch.py] param {k} = {node_parameters[k]}")

    # 预留 launch 参数（目前 cam_topic_arg 只是声明，实际 node_parameters 里已经写死了 cam_topic）
    # 如果你希望通过命令行覆盖，可进一步改成 LaunchConfiguration 绑定。
    cam_topic_arg = DeclareLaunchArgument(
        'cam_topic',
        default_value='/cam0/image_raw',
        description='Camera topic to subscribe to'
    )

    # SVO 节点
    svo_node = Node(
        package='svo_ros',
        executable='vo',
        name='svo',
        output='screen',
        parameters=[node_parameters],
    )

    return LaunchDescription([
        cam_topic_arg,
        svo_node,
    ])
