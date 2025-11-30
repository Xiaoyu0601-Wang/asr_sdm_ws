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
### Related problems：
#### 1.After the host runs the launch file

When the host runs the launch file, the ROS 2 node starts and publishes the relevant topic. If the host is configured correctly, Radxa 3E should be able to subscribe to these topics via ROS 2's communication mechanism without having to run the launch file again on Radxa 3E.

#### 2.Radxa 3E only needs to make sure the following:
- ROS 2 is installed and configured correctly
- The Radxa 3E is able to connect to the network where the host is located.
- Radxa 3E is able to discover topics published by the host.

#### 3.Distributed communication for ROS 2
The nodes of ROS 2 use DDS for distributed communication. After the node on the host is started, it automatically broadcasts topic information on the network, and Radxa 3E only needs to discover these topics through DDS.

If the host and Radxa 3E's ROS 2 environment are configured correctly, they should be able to automatically discover each other's nodes and topics.

#### 4.Network configuration:
Ensure that the Radxa 3E is able to access the host over the network. For example, whether the IP addresses of the host and Radxa 3E are in the same subnet, and whether the network is unblocked. You can use the ping command to test it.

####  5.ROS 2 configuration check
- **ROS_DOMAIN_ID**: If two devices use different **ROS_DOMAIN_ID**, they may not be able to discover each other in the same network. Make sure both devices use the same ROS_DOMAIN_ID, otherwise explicit settings are required:

          export ROS_DOMAIN_ID=<Your Host DOMAIN_ID>
          source ~/.bashrc
### 2.When the Radxa 3E runs the launch file,  enter the rqt command on the host, and the terminal will display,and it will not automatically show the camera feed for you.

          user@user:~/ros2_ws$ rqt
          QSocketNotifier: Can only be used with threads started with QThread
          Wayland does not support QWindow::requestActivate()
    
### Solution:Radxa 3E only needs to make sure the following:

#### Step 1:check if the host can "see" the camera topic on the board

          source /opt/ros/humble/setup.bash

  **Then look at the topic list on the host:**

          ros2 topic list
  **You can see something like the following on the host**

          /camera/camera/color/camera_info
          /camera/camera/color/image_rect_raw
          /camera/camera/depth/image_rect_raw
          ...


**NEXT**
#### Step 2:See if there is any data flowing on this topic
          ros2 topic echo /camera/camera/color/image_rect_raw --qos-profile sensor_data

**If a large lump keeps printing on the screen header / height / width / encoding / data , it means:**

**The image data of the Radxa → host is already being transmitted  normally**

### Finally
#### Step 3:Then use rqt to open the Image View + Select topic correctly


Turn on rqt on the host (add a prefix for Wayland):

          env QT_QPA_PLATFORM=xcb rqt
          Plugins → Visualization → Image View

Just cutting the backend of Qt to X11, the main function is:
- Avoid window focus/pinning issues in a few cases
- Eliminate warning these warnings

