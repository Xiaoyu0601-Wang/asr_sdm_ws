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