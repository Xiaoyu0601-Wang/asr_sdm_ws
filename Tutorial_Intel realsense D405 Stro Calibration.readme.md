**此双目相机Intel realsense D405校正测试用于ubuntu24.03.live-server-amd-rolling端**

This tutorial cover using the node to calibrate a stereo camera(Intel realsense D405) with a left and right image over ROS 2.

**Before Starting** 
 Make sure that you have the following:
- A large checkerboard (8x6) or checkerboard (7x6) with known dimensions. This tutorial uses a 8x6 checkerboard with 108mm squares.
  -  [Checkerboards template](https://markhedleyjones.com/project/calibration-checkbroard-collection)
- A well lit 5m x 5m area clear of obstructions and check board patterns a stereo camera publishing left and right images over ROS (if you want to use two independent cameras as a stereo camera, you must make sure the images have identical time stamps).

- Install ROS2 and the camera_calibration package

       sudo apt install ros-rolling-camera-calibration
  

**Step 1:Launch the camera node**


 - Only enable infra1 + infra2 (recommended for stereo calibration
 
       ros2 launch realsense2_camera rs_launch.py \
       enable_color:=false enable_depth:=false \
       enable_infra:=false \
       enable_infra1:=true enable_infra2:=true \
       depth_module.infra_profile:=848x480x30 \
       depth_module.infra1_format:=Y8 \
       depth_module.infra2_format:=Y8
 - **Note:camera_calibration里调用了self.get_logger().warn(...),当前rclpy里RcutilsLogger.warn不存在**
 - Solution:Apply a patch
 
       # 找到出问题的地方（可选）
       grep -n "get_logger().warn" -n /opt/ros/rolling/lib/python3.12/site-packages/camera_calibration/camera_calibrator.py
       # 把 warn() 改成 warning()
       sudo sed -i 's/\.warn(/\.warning(/g' /opt/ros/rolling/lib/python3.12/site-packages/camera_calibration/camera_calibrator.py

**Step 2:Verify the topics after launch**

       ros2 topic list | grep -E "infra1|infra2"
       ros2 topic hz /camera/camera/infra1/image_rect_raw
       ros2 topic hz /camera/camera/infra2/image_rect_raw
   **example outcome**
   
	/camera/camera/extrinsics/depth_to_infra1
	/camera/camera/extrinsics/depth_to_infra2
	/camera/camera/infra1/camera_info
	/camera/camera/infra1/image_rect_raw
	/camera/camera/infra1/metadata
	/camera/camera/infra2/camera_info
	/camera/camera/infra2/image_rect_raw
	/camera/camera/infra2/metadata
	average rate: 20.715
	min: 0.032s max: 0.134s std dev: 0.03588s window: 20
	average rate: 20.538
	min: 0.032s max: 0.134s std dev: 0.03619s window: 39
	average rate: 19.553
	min: 0.032s max: 0.167s std dev: 0.03933s window: 58
	average rate: 21.428
	min: 0.032s max: 0.167s std dev: 0.03480s window: 85
	average rate: 23.155
	min: 0.032s max: 0.167s std dev: 0.03049s window: 115

也可以使用rqt观察

**Step 3:Start the Calibration**
To start the calibration you will need to remap the image topics that will be calibrated:

    ros2 run camera_calibration cameracalibrator \
    --no-service-check --approximate 0.1 --size 8x6 --square 0.025 \
	  --ros-args \
	  -r left:=/camera/camera/infra1/image_rect_raw \
	  -r right:=/camera/camera/infra2/image_rect_raw
