# ASR SDM Controller

本文件夹实现了基于文献中的蛇形机器人前端单元跟随控制方法。
原文链接：[Modeling and Control of a Snake-Like Robot Using the Screw-Drive Mechanism](https://ieeexplore.ieee.org/document/9428758)

## 节点：`asr_sdm_controller`

该节点订阅一个 `geometry_msgs/msg/Twist` 消息，并根据运动学模型计算出每个螺旋驱动单元的关节角度和螺旋速度，最终以 `asr_sdm_control_msgs/msg/ControlCmd` 消息格式发布。

## 前置依赖

本软件包依赖 `c-periphery` 库。在构建前，请先从源代码编译并安装它：

```bash
# 1. 下载源代码
git clone https://github.com/vsergeev/c-periphery.git

# 2. 创建构建目录并使用 CMake 配置
cd c-periphery
mkdir build
cd build
cmake ..

# 3. 编译
make

# 4. 安装到系统中
sudo make install

# 5. 更新系统库缓存
sudo ldconfig
```

## 如何构建

在您的工作区根目录运行：

```bash
colcon build --packages-select asr_sdm_controller
```

### 如何运行

我们提供了一个 launch 文件来方便地启动节点并加载所有参数。

```bash
source install/setup.bash
ros2 launch asr_sdm_controller head_tracker.launch.py
```

### 订阅的话题

*   `/cmd_vel` (`geometry_msgs/msg/Twist`)
    *   用于控制机器人头部运动的线速度和角速度指令。`linear.x` 用于前进/后退，`angular.z` 用于左/右转向。

### 发布的话题

*   `/control_cmd` (`asr_sdm_control_msgs/msg/ControlCmd`)
    *   发送给硬件接口的底层控制指令，包含每个单元的螺旋速度和关节角度。
*   `~/output/controller/heartbeat` (`std_msgs/msg/String`)
    *   用于监控节点是否存活的心跳消息。

### 参数

所有参数都在 `config/head_tracker_params.yaml` 文件中定义：

*   `link_front_length`: 关节到单元前端的距离 (m)
*   `link_rear_length`: 关节到单元后端的距离 (m)
*   `screw_radius`: 螺旋轮半径 (m)
*   `joint_limit`: 关节最大偏转角 (rad)
*   `control_rate_hz`: 控制循环频率 (Hz)
*   `cmd_timeout`: 指令超时时间 (s)，若超过此时长未收到新指令，则速度归零。
*   `max_linear_speed`: 最大线速度 (m/s)
*   `max_angular_speed`: 最大角速度 (rad/s)
*   `screw_velocity_scale`: 螺旋角速度(rad/s)到电机指令的缩放系数。
*   `max_screw_command`: 最大螺旋电机指令值。
*   `joint_angle_scale`: 关节角度(rad)到电机指令的缩放系数。
*   `heartbeat_period_ms`: 心跳消息周期 (ms)
*   `helix_alpha`: 包含4个螺旋轮螺旋角的数组 (rad)
*   `unit_ids`: 4个单元的ID，与硬件通信对应。
*   `joint_index_map`: 关节索引映射，-1表示该单元无关节。
