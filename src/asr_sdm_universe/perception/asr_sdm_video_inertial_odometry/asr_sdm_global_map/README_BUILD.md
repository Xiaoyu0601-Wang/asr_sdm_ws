# asr_sdm_global_map 编译说明

## 依赖要求

这个包需要 **GTSAM (Georgia Tech Smoothing and Mapping)** 库。

## 安装 GTSAM

### 方法 1: 使用 apt 安装（推荐）

```bash
sudo apt-get update
sudo apt-get install -y libgtsam-dev libgtsam4
```

### 方法 2: 从源码编译

如果需要特定版本的 GTSAM，可以从源码编译：

```bash
git clone https://github.com/borglab/gtsam.git
cd gtsam
mkdir build && cd build
cmake ..
make -j$(nproc)
sudo make install
```

## 编译此包

安装 GTSAM 后，移除 CATKIN_IGNORE 文件（如果存在）：

```bash
cd /home/lxy/asr_sdm_ws/src/asr_sdm_universe/perception/asr_sdm_video_inertial_odometry/asr_sdm_global_map
rm -f CATKIN_IGNORE
```

然后编译：

```bash
cd /home/lxy/asr_sdm_ws
colcon build --packages-select asr_sdm_global_map
```

## 如果不需要此包

如果您不需要全局地图功能，可以保持 CATKIN_IGNORE 文件，编译系统会自动跳过此包。

## 功能说明

此包提供基于 GTSAM 的全局地图优化功能，用于视觉惯性里程计的后端优化。

