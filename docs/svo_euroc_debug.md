# SVO (svo_ros/vo) 跑 EuRoC MH_01_easy_ros2 调试复盘（含代码改动）

> 目标：使用 `svo_ros/vo`（单目）在 ROS2 下对 `MH_01_easy_ros2` rosbag **稳定跟踪**，并能解释/复现从“初始化很强但很快跟丢”到“全程跟踪”的调试路径。
>
> 本文记录了从现象→定位→修复的完整过程，并列出所有关键代码/参数改动，方便学习与溯源。

---

## 0. 背景与现象

### 0.1 离线测试 OK、ROS 播放差
- 离线测试（例如 `test_pipeline_euroc.cpp`）在调参后可达到较高跟踪成功率。
- 但在 ROS2 `ros2 bag play datasheet/MH_01_easy_ros2` 流程中：
  - 初始化阶段 `Init: KLT tracked ~400+ features` 往往正常
  - 进入跟踪后大量出现：
    - `Lost XX features!`
    - `Relocalizing frame`
    - `Not enough matched features.`
  - 视觉上表现为：特征点看起来很少、很快丢失、频繁重定位。

### 0.2 关键判断
初始化 KLT 很强 ⇒ 图像内容/编码/特征提取能力基本没问题；问题更可能在：
- 参数是否真的作用到 **SVO 核心 `svo::Config`**
- 匹配/优化阈值过严导致大量点被剔除
- depth filter 的 epipolar search 深度区间发散，导致深度更新被跳过

---

## 1. 诊断步骤与结论（按时间顺序）

### 1.1 基础连通性确认
- `ros2 node list` 确认 `/svo` 是否存在
- `ros2 topic list -t` 确认 `/cam0/image_raw` 存在
- `ros2 bag info datasheet/MH_01_easy_ros2` 确认 bag topic

结论：节点在跑、能收到图像（因为初始化能 KLT 400+）。

### 1.2 排除图像格式问题
- `ros2 topic echo /cam0/image_raw --field encoding --once`
- 输出为 `mono8`

结论：排除 RGB/BGR 转灰度错误导致特征少。

### 1.3 相机模型与畸变（“路线1”：让相机模型支持畸变）
EuRoC cam0 原始为 radial-tangential 畸变；原先 `camera_euroc.yaml` 只有内参。

目标：让 `svo_ros/vo` 读取畸变参数，并构建带畸变的 `vk::PinholeCamera`。

**代码改动 1：`svo_ros/src/vo_node.cpp` 使用 vikit camera_loader 构建相机**
- 由手工 `new vk::PinholeCamera(width,height,fx,fy,cx,cy)`
- 改为 `vk::camera_loader::loadFromRosNode(this, "", cam_)`

这样可支持读取：`cam_d0..cam_d3`（OpenCV radtan：[k1,k2,p1,p2]）。

**参数改动 1：`svo_ros/param/camera_euroc.yaml` 补齐畸变参数**
- `cam_model: Pinhole`（注意大小写，camera_loader 识别的是 `Pinhole`）
- 添加：
  - `cam_d0 = k1`
  - `cam_d1 = k2`
  - `cam_d2 = p1`
  - `cam_d3 = p2`
  - `cam_d4 = k3(0)`

> 结论：畸变参数可进入节点；但后续实验表明，即使置零畸变，系统仍可能发散，因此根因不只在畸变。

---

## 2. “参数生效链路”问题（非常关键）

### 2.1 现象
你会遇到“我明明改了 yaml，但 `/svo` 里 `Parameter not set` 或仍然用默认值”。

主要原因：
1) `test_euroc.launch.py` 曾多次变更；而 `svo_ros/vo` 这个节点 **读取的是平铺参数（flat key）**。
2) `vo_euroc_stable.yaml` 在调试过程中出现过两种格式：
   - nested: `svo: { ros__parameters: {...} }`
   - flat: `grid_size: ...` 直接顶层

### 2.2 Launch 修复
**代码改动 2：`svo_ros/launch/test_euroc.launch.py`**
- 加入 `_load_yaml()`
- 自动兼容 nested/flat 两种 VO yaml 格式
- 启动时打印关键参数（确认最终传入 Node 的值）

输出形如：
```
[test_euroc.launch.py] param grid_size = 25
...
```

---

## 3. 将 ROS 参数写回 SVO 核心 Config（关键修复）

### 3.1 根因
`svo_ros/vo` 内部核心使用单例 `svo::Config`。

原始 `vo_node.cpp` 在 non-euroc 分支里只写回了少量：
- `grid_size / max_fts / triang_min_corner_score / n_pyr_levels`

但你在 yaml 里调的很多关键鲁棒性阈值（reproj/poseopt/quality/kf/zmssd 等）**没写回 Config** ⇒ SVO 仍用默认值（通常更严格）⇒ 大量 outlier ⇒ Relocalize。

### 3.2 修复：扩展 non-euroc 分支写回 Config
**代码改动 3：`svo_ros/src/vo_node.cpp`**
在 non-euroc 分支新增写回：
- `svo::Config::reprojThresh()` ← `reproj_thresh`
- `svo::Config::poseOptimThresh()` ← `poseoptim_thresh`
- `svo::Config::qualityMinFts()` ← `quality_min_fts`
- `svo::Config::qualityMaxFtsDrop()` ← `quality_max_drop_fts`
- `svo::Config::kfSelectMinDist()` ← `kfselect_mindist`
- `svo::Config::maxNKfs()` ← `max_n_kfs`
- `svo::Config::mapScale()` ← `map_scale`
- `svo::Config::patchMatchThresholdFactor()` ← `patch_match_thresh_factor`

并增加了权威日志：

**代码改动 4：`vo_node.cpp` 增加 `SVO Config(final)` 打印**
输出真实生效的核心值：
```
SVO Config(final): grid=... max_fts=... ... reproj=... poseopt=... zmssd_factor=... klt_levels=[..] subpix_iter=...
```

---

## 4. 让 KLT 更能扛大运动 + 提升亚像素对齐

### 4.1 现象
`SVO Config(final)` 显示 `klt_levels=[2..4]`，对 EuRoC 部分帧间位移较大场景不友好。

### 4.2 修复
**代码改动 5：`svo_ros/src/vo_node.cpp`**
新增参数并写回 Config：
- `klt_min_level` → `svo::Config::kltMinLevel()`
- `klt_max_level` → `svo::Config::kltMaxLevel()`
- `subpix_n_iter` → `svo::Config::subpixNIter()`

**参数改动 2：`svo_ros/param/vo_euroc_stable.yaml`**
示例：
```yaml
klt_min_level: 0
klt_max_level: 4
subpix_n_iter: 30
```

---

## 5. 最终根因：Depth Filter epipolar search 深度区间发散

### 5.1 现象（最关键）
日志出现大量：
- `skip epipolar search: ... px_lenght=xxxx ... d_max=100000000...`
- 甚至 `px_lenght`、`evaluations` 变成天文数字

这意味着：seed 的深度区间（d_min/d_max）发散 ⇒ epipolar line 太长 ⇒ `n_steps` 超过上限（默认 1000） ⇒ 直接 skip ⇒ depth filter 没观测更新 ⇒ 地图崩 ⇒ 进入 relocalize 死循环。

### 5.2 定位打印来源
字符串来自：
- `svo/src/matcher.cpp`，函数：`Matcher::findEpipolarMatchDirect(...)`

### 5.3 修复策略（工程护栏）：对 epipolar search 深度范围做 clamp
为了让 depth filter 能持续获得观测并收敛，必须保证搜索区间不发散到不可计算。

**代码改动 6：`svo/src/matcher.cpp`**
在 `findEpipolarMatchDirect` 入口加入深度范围护栏：
- clamp 到 0.2m ~ 30m：
  - `d_min_c = max(d_min, 0.2)`
  - `d_max_c = min(d_max, 30.0)`
- 使用 `d_min_c/d_max_c` 计算 epipolar line 端点 A/B

效果：
- `skip epipolar search` 从大量、爆炸级别 → 少量且可控
- 跟踪从“很快丢失” → **可以全程跟踪**

> 注意：warning 中 `d_max=1e8` 仍可能出现，是上游传入值；但实际搜索端点已用 clamp 后的区间计算。

---

## 6. 当前推荐配置（概览）

### 6.1 相机参数：`camera_euroc.yaml`
- `cam_model: Pinhole`
- `cam_fx/cam_fy/cam_cx/cam_cy`
- `cam_d0..cam_d3`（radtan）

### 6.2 VO 参数：`vo_euroc_stable.yaml`（flat）
典型关键项：
- `grid_size / max_fts / n_pyr_levels / triang_min_corner_score`
- `reproj_thresh / poseoptim_thresh`
- `quality_min_fts / quality_max_drop_fts`
- `kfselect_mindist / max_n_kfs / map_scale`
- `patch_match_thresh_factor`
- `klt_min_level / klt_max_level / subpix_n_iter`

---

## 7. 运行与验证 checklist

1) 启动：
```bash
source /opt/ros/jazzy/setup.bash
source ~/svo/asr_sdm_ws/install/setup.bash
ros2 launch svo_ros test_euroc.launch.py
```

2) 确认启动日志：
- `SVO Config(final)` 是否包含期望值
- `klt_levels=[0..4]`、`subpix_iter=30` 是否生效

3) 播放 bag：
```bash
ros2 bag play ~/svo/asr_sdm_ws/datasheet/MH_01_easy_ros2 --clock --rate 1.0
```

4) 观察：
- `skip epipolar search` 是否只剩少量
- 是否还能 “全程跟踪”

---

## 8. 关键改动文件清单（便于 git diff / 溯源）
- `src/.../svo_ros/src/vo_node.cpp`
  - 相机创建改用 `vk::camera_loader::loadFromRosNode`
  - non-euroc 分支写回 `svo::Config` 的完整参数集
  - 新增 `SVO Config(final)` 日志
  - 新增 KLT/subpix 参数写回

- `src/.../svo_ros/param/camera_euroc.yaml`
  - `cam_model: Pinhole`
  - 增加 `cam_d0..cam_d3`

- `src/.../svo_ros/param/vo_euroc_stable.yaml`
  - flat 参数结构 + 新增鲁棒性/KLT/subpix 参数

- `src/.../svo_ros/launch/test_euroc.launch.py`
  - 兼容 nested/flat yaml
  - 启动时打印关键参数

- `src/.../svo/src/matcher.cpp`
  - epipolar search 深度范围 clamp (0.2~30m)
  - skip epipolar search 日志带 limit 信息

---

## 9. 后续可改进（非必须）
- 将 matcher.cpp 的深度 clamp 从硬编码改为 Config 参数（可在 ROS yaml 中配置不同场景）
- 将 `options_.max_epi_search_steps` 暴露为可配置（取舍：更少 skip vs 更高计算量）
- 深入排查上游为什么会产生 `d_max=1e8` 的 seed 区间（根因可能在 seed 初始化方差/尺度估计）

---

## 10. 关键代码改动（可直接对照 / 复制）

> 注：以下片段是“概念上关键”的差异点，便于你快速定位；实际以仓库当前文件为准。

### 10.1 `svo_ros/src/vo_node.cpp`：相机加载改为 camera_loader（支持畸变）

**原先（简化示意）**
```cpp
cam_model = vk::getParam<std::string>(this, "cam_model", "PINHOLE");
...
cam_ = new vk::PinholeCamera(width, height, fx, fy, cx, cy);
```

**现在（核心点）**
```cpp
bool ok = vk::camera_loader::loadFromRosNode(this, "", cam_);
if(!ok || cam_ == nullptr) throw ...;
```

### 10.2 `svo_ros/src/vo_node.cpp`：non-euroc 分支将 ROS 参数写回 `svo::Config`

**新增写回项（示意）**
```cpp
svo::Config::reprojThresh() = vk::getParam<double>(this, "reproj_thresh", svo::Config::reprojThresh());
svo::Config::poseOptimThresh() = vk::getParam<double>(this, "poseoptim_thresh", svo::Config::poseOptimThresh());
svo::Config::qualityMinFts() = (size_t)vk::getParam<int>(this, "quality_min_fts", (int)svo::Config::qualityMinFts());
...
svo::Config::patchMatchThresholdFactor() = vk::getParam<double>(this, "patch_match_thresh_factor", svo::Config::patchMatchThresholdFactor());
```

### 10.3 `svo_ros/src/vo_node.cpp`：增加 KLT / subpix 参数写回
```cpp
svo::Config::kltMinLevel() = (size_t)vk::getParam<int>(this, "klt_min_level", (int)svo::Config::kltMinLevel());
svo::Config::kltMaxLevel() = (size_t)vk::getParam<int>(this, "klt_max_level", (int)svo::Config::kltMaxLevel());
svo::Config::subpixNIter() = (size_t)vk::getParam<int>(this, "subpix_n_iter", (int)svo::Config::subpixNIter());
```

### 10.4 `svo_ros/src/vo_node.cpp`：增加 `SVO Config(final)` 权威打印
```cpp
RCLCPP_INFO(this->get_logger(),
  "SVO Config(final): grid=%zu max_fts=%zu ... klt_levels=[%zu..%zu] zmssd_factor=%.2f subpix_iter=%zu",
  ...);
```

### 10.5 `svo_ros/launch/test_euroc.launch.py`：兼容 nested/flat + 启动打印

关键点：
- `vo_euroc_stable.yaml` 如果是 flat 结构直接 merge
- 如果是 nested（svo/ros__parameters）则取出该层 merge

并在 launch 侧打印关键参数，方便确认 install/share 下的 yaml 是否生效。

### 10.6 `svo/src/matcher.cpp`：对 epipolar search 深度范围做 0.2~30m clamp（最终关键）

位置：`Matcher::findEpipolarMatchDirect(...)`

```cpp
constexpr double kDepthMinClamp = 0.2;
constexpr double kDepthMaxClamp = 30.0;
const double d_min_c = std::max(d_min, kDepthMinClamp);
const double d_max_c = std::min(d_max, kDepthMaxClamp);
if (!(d_max_c > d_min_c)) return false;

Vector2d A = vk::project2d(T_cur_ref * (ref_ftr.f*d_min_c));
Vector2d B = vk::project2d(T_cur_ref * (ref_ftr.f*d_max_c));
```

**解释：**
- 避免 seed 深度区间发散导致 `epi_length_` 和 `n_steps` 爆炸
- 防止 `max_epi_search_steps` 频繁触发，造成 depth filter 无法更新

---

## 11. 最终推荐参数快照（便于复现）

### 11.1 `vo_euroc_stable.yaml`（建议只保留这一份，并保持 flat 结构）
```yaml
grid_size: 25
max_fts: 400
n_pyr_levels: 4
triang_min_corner_score: 10.0

max_n_kfs: 40
map_scale: 3.0
loba_num_iter: 0

reproj_thresh: 5.0
poseoptim_thresh: 5.0
quality_min_fts: 20
quality_max_drop_fts: 120
kfselect_mindist: 0.02
patch_match_thresh_factor: 1.4

klt_min_level: 0
klt_max_level: 4
subpix_n_iter: 30
```

### 11.2 `camera_euroc.yaml`（示意；请以实际写入值为准）
```yaml
cam_model: Pinhole
cam_width: 752
cam_height: 480
cam_fx: 458.6548807207614
cam_fy: 457.2966964634893
cam_cx: 367.2158039615726
cam_cy: 248.37534060980727
cam_d0: -0.28340811217029355
cam_d1: 0.07395907389290132
cam_d2: 0.00019359502856909603
cam_d3: 1.7618711454538528e-05
cam_d4: 0.0
```

---

## 12. 常见坑（快速排错）

1) **launch 读的是 install/share 下的 yaml**
- 如果你改了 src 下的 yaml，但没重新 build/source，就会一直用旧参数。

2) **参数类型要匹配**
- 例如 `triang_min_corner_score` 在节点里是 double，yaml 里写 `10` 会报“int 不能覆盖 double”，要写 `10.0`。

3) **仅仅把参数作为 ROS param 传入不够**
- 必须确认参数是否写入 `svo::Config`（用 `SVO Config(final)` 验证）。

---

## 13. 回滚与对比
- 想对比 clamp 的影响：回滚 `svo/src/matcher.cpp` 中 d_min/d_max clamp 段。
- 想对比畸变影响：将 `camera_euroc.yaml` 的 d0..d3 临时置零。

