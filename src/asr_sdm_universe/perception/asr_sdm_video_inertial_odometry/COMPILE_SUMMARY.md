# asr_sdm_video_inertial_odometry 编译总结

## 编译日期
2025-11-16

## 编译状态
✅ **编译成功**

## 已成功编译的包

### 基础依赖包
1. ✅ **eigen_checks** - Eigen 检查工具
2. ✅ **fast** - FAST 角点检测库
3. ✅ **minkindr** - 最小运动学和动力学表示库
4. ✅ **svo_cmake** - SVO CMake 配置包

### Video Kit 包
5. ✅ **video_kit_common** - 视频工具通用库
6. ✅ **video_kit_cameras** - 相机模型库
7. ✅ **video_kit_ros2** - ROS2 接口
8. ✅ **video_kit_solver** - 求解器库
9. ✅ **video_kit_py** - Python 接口

### VIO 核心包
10. ✅ **asr_sdm_vio_common** - VIO 通用库
11. ✅ **asr_sdm_vio_test_utils** - VIO 测试工具
12. ✅ **asr_sdm_vio_imu** - IMU 处理模块
13. ✅ **asr_sdm_vio_direct** - 直接法视觉里程计
14. ✅ **asr_sdm_vio_img_align** - 图像对齐模块
15. ✅ **asr_sdm_vio_tracker** - 特征跟踪器

### VIO 高级功能包
16. ✅ **asr_sdm_rpg_common** - RPG 通用库
17. ✅ **asr_sdm_vio_online_loopclosing** - 在线回环检测
18. ✅ **asr_sdm_vio_pgo** - 位姿图优化

## 未编译的包

### asr_sdm_vio_global_map
⚠️ **状态**: 已跳过（CATKIN_IGNORE）

**原因**: 需要 GTSAM (Georgia Tech Smoothing and Mapping) 库

**解决方案**:
1. 安装 GTSAM:
   ```bash
   sudo apt-get install -y libgtsam-dev libgtsam4
   ```

2. 移除 CATKIN_IGNORE 文件:
   ```bash
   cd /home/lxy/asr_sdm_ws/src/asr_sdm_universe/perception/asr_sdm_video_inertial_odometry/asr_sdm_vio_global_map
   rm CATKIN_IGNORE
   ```

3. 重新编译:
   ```bash
   cd /home/lxy/asr_sdm_ws
   colcon build --packages-select asr_sdm_vio_global_map
   ```

**详细说明**: 请参阅 `asr_sdm_vio_global_map/README_BUILD.md`

## 编译警告

### asr_sdm_vio_imu
- 警告类型: `#pragma diagnostic` 指令未被识别
- 影响: 无，仅为编译器兼容性警告
- 状态: 可以忽略

### asr_sdm_vio_online_loopclosing
- 警告类型: 运行时库路径冲突（libcurl.so.4）
- 原因: Anaconda 环境与系统库路径冲突
- 影响: 轻微，可能需要在运行时注意
- 状态: 可以忽略

## 编译命令

### 编译所有 VIO 包
```bash
cd /home/lxy/asr_sdm_ws
colcon build --packages-up-to asr_sdm_vio_online_loopclosing asr_sdm_vio_pgo asr_sdm_vio_tracker
```

### 编译单个包
```bash
cd /home/lxy/asr_sdm_ws
colcon build --packages-select <package_name>
```

### 清理并重新编译
```bash
cd /home/lxy/asr_sdm_ws
rm -rf build/ install/ log/
colcon build
```

## 环境设置

编译完成后，需要 source 环境：
```bash
source /home/lxy/asr_sdm_ws/install/setup.bash
```

建议将此命令添加到 `~/.bashrc`:
```bash
echo "source /home/lxy/asr_sdm_ws/install/setup.bash" >> ~/.bashrc
```

## 包依赖关系

```
svo_cmake, eigen_checks, fast, minkindr
    ↓
video_kit_common, video_kit_solver
    ↓
video_kit_cameras, video_kit_ros2
    ↓
asr_sdm_vio_common
    ↓
asr_sdm_vio_test_utils, asr_sdm_vio_imu
    ↓
asr_sdm_vio_direct, asr_sdm_vio_img_align
    ↓
asr_sdm_vio_tracker, asr_sdm_rpg_common
    ↓
asr_sdm_vio_online_loopclosing, asr_sdm_vio_pgo
    ↓
[asr_sdm_vio_global_map] (需要 GTSAM)
```

## 下一步

1. ✅ 所有核心 VIO 功能包已成功编译
2. ⚠️ 如需全局地图优化功能，请安装 GTSAM 并编译 asr_sdm_vio_global_map
3. ✅ 可以开始使用 VIO 系统进行开发和测试

## 技术支持

如遇到编译问题，请检查：
1. ROS2 Jazzy 是否正确安装
2. 所有系统依赖是否安装（OpenCV, Eigen3, glog 等）
3. 编译日志中的具体错误信息

---
*编译总结自动生成*

