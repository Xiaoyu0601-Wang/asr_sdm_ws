本安装测试用于ubuntu24.03.live-server-amd端
**#Installation on Ubuntu** 
##Step 1:Install the ROS2 distribution*
- Ubuntu 24.04 
  -  [ROS2 Rolling](https://docs.ros.org/en/rolling/Installation/Ubuntu-Install-Debs.html)
  -  [ROS2 Jazzy](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)

在ubuntu终端中输入`printenv ROS_DISTRO`
若输出为`rolling`则证明安装成功。

##Step 2:Install latest Intel® RealSense™ SDK 2.0

 - **Option 1: Install librealsense2 debian package from Intel servers**
     - Jetson user - use the [ Jetson Installation Guide](https://github.com/IntelRealSense/librealsense/blob/master/doc/installation_jetson.md)
      - Otherwise, install from[[Linux Debian Installation Guide](https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md#installing-the-packages)]
         -   In this case treat yourself as a developer: make sure to follow the instructions to also install librealsense2-dev and librealsense2-dkms packages
 - **Option 2: Install librealsense2 (without graphical tools and examples) debian package from ROS servers (Foxy EOL distro is not supported by this option)**:
 - Configure your Ubuntu repositories
 - Install all realsense ROS packages by `sudo apt install ros-<ROS_DISTRO>-librealsense2*`
     - For example, for Humble distro: sudo apt install ros-humble-librealsense2*
 - Option 3: Build from source
 - Download the latest [Intel® RealSense™ SDK 2.0](https://github.com/IntelRealSense/librealsense)
 - Follow the instructions under [[Linux Installation](https://github.com/IntelRealSense/librealsense/blob/master/doc/installation.md)]
 - **注意：执行到Run Intel Realsense permissions script from _librealsense2_ root directory
                         cd librealsense
                          ./scripts/setup_udev_rules.sh
    时会出现`v4l2-ctl not found, install with: sudo apt install v4l-utils问题`，继续输入`sudo apt install v4l-utils`**
    **在终端输入realsense-viewer以测试成功安装。**

##Step 3: Install ROS Wrapper for Intel® RealSense™ cameras


   Install from source
 

    mkdir -p ~/ros2_ws/src
  cd ~/ros2_ws/src/
- Clone the latest ROS Wrapper for Intel® RealSense™ cameras from here into '~/ros2_ws/src/'
 

       git clone https://github.com/IntelRealSense/realsense-ros.git -b ros2-master
       cd ~/ros2_ws
- Install dependencies
 

         sudo apt-get install python3-rosdep -y
         sudo rosdep init # "sudo rosdep init --include-eol-distros" for Foxy        and earlier
         rosdep update # "sudo rosdep update --include-eol-distros" for          Foxy and earlier
         rosdep install -i --from-path src --rosdistro $ROS_DISTRO --skip-keys=librealsense2 -y

-  Build
        

          colcon build

- Source environment 

          ROS_DISTRO=<YOUR_SYSTEM_ROS_DISTRO>  # set your    ROS_DISTRO: kilted, jazzy, iron, humble, foxy
          source /opt/ros/$ROS_DISTRO/setup.bash
          cd ~/ros2_ws
          . install/local_setup.bash
 - 验证：
 
     `rviz2`
     
     观察pointclouds信息
  - - -         

