# Installation

## Dependency
```sh
sudo apt-get -y install ros-$ROS_DISTRO-sophus libgoogle-glog-dev
```

## Build
```sh
colcon build --symlink-install --parallel-workers 8
```

# Run Instructions

### Run using ROS2 (Rolling/Jazzy)



### SVO Visual Odometry demo (rosbag + RViz2)
This section shows how to run the SVO visual odometry node with the included rosbag and visualize results in RViz2.

Prerequisites
`rm -rf build install log`
- Workspace has been built: `colcon build --packages-up-to svo_ros`
- Source the workspace: `source install/setup.bash`
- The demo bag exists at: `datasheet/airground_rig_s3_ros2/airground_rig_s3_ros2.db3`
- RViz2 config file: `src/asr_sdm_universe/perception/asr_sdm_video_inertial_odometry/asr_sdm_svo/svo_ros/rviz_config.rviz`

Recommended: use three terminals and source the workspace in each
```sh
source install/setup.bash
```

Terminal 1 — start SVO
```sh
ros2 launch svo_ros test_rig3.launch.py
ros2 launch svo_ros test_euroc.launch.py
ros2 launch svo_ros vio_euroc.launch.py
```

Terminal 2 — play rosbag with simulated clock, loop
```sh
ros2 bag play datasheet/airground_rig_s3_ros2
ros2 bag play ~/svo/asr_sdm_ws/datasheet/MH_01_easy_ros2 --clock --rate 1.0
```

Terminal 3 — start RViz2 with the provided config and use simulated time
```sh
rviz2 -d src/asr_sdm_universe/perception/asr_sdm_video_inertial_odometry/asr_sdm_svo/svo_ros/rviz_config.rviz 
```

What you should see
- Nodes: `/svo`, `/rosbag2_player`
- Topics: `/camera/image_raw`, `/svo/image`, `/svo/keyframes`, `/svo/points`, `/svo/pose`, `/tf`, `/clock`

Quick checks
```sh
# list nodes
ros2 node list

# list topics
ros2 topic list | sort

# SVO node info (publishers/subscribers)
ros2 node info /svo

# check publishing rates
ros2 topic hz /svo/image
ros2 topic hz /svo/pose
```

Stop processes
```sh
# stop rosbag player
pkill -f "ros2 bag play"

# stop SVO launch
pkill -f "ros2 launch svo_ros test_rig3.launch.py"

# stop RViz2
pkill -f "rviz2 -d .*rviz_config.rviz"
```

Notes
- The SVO launch (`test_rig3.launch.py`) subscribes to `/camera/image_raw`, which matches the bag topics, so no remapping is needed.
- The rosbag is played with `--clock`. RViz2 is started with `use_sim_time:=true` so the timeline follows the bag.
- If you prefer a single combined launch that starts SVO, rosbag, and RViz together, open an issue or request and we can add an integrated launch file.
