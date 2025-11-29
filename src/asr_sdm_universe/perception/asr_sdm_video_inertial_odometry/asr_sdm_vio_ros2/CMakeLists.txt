cmake_minimum_required(VERSION 3.8)
project(asr_sdm_vio_ros2)

# ROS2 ament package providing a minimal frontend node and resources.

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(video_kit_cameras REQUIRED)
find_package(CURL REQUIRED)
find_package(message_filters REQUIRED)
find_package(asr_sdm_vio REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(asr_sdm_vio_ceres_backend REQUIRED)

# Minimal executable that subscribes to image and imu topics
add_executable(minimal_vio_frontend src/minimal_vio_frontend.cpp)
ament_target_dependencies(minimal_vio_frontend rclcpp sensor_msgs)
install(TARGETS minimal_vio_frontend
  DESTINATION lib/${PROJECT_NAME}
)

# TF broadcaster for PoseStamped("Rig") -> TF map->rig
add_executable(rig_pose_to_tf src/rig_pose_to_tf.cpp)
ament_target_dependencies(rig_pose_to_tf rclcpp geometry_msgs tf2_ros)
install(TARGETS rig_pose_to_tf
  DESTINATION lib/${PROJECT_NAME}
)

# VIO frontend ROS2 skeleton (loads camera, wires subs)
add_executable(vio_frontend_ros2 src/vio_frontend_ros2.cpp)
ament_target_dependencies(vio_frontend_ros2 rclcpp sensor_msgs cv_bridge ament_index_cpp video_kit_cameras asr_sdm_vio)
# Link cv_bridge as imported target to ensure include dirs are set
# (No explicit OpenCV link; cv_bridge handles it)
target_link_libraries(vio_frontend_ros2 cv_bridge::cv_bridge CURL::libcurl ${asr_sdm_vio_LIBRARIES})
install(TARGETS vio_frontend_ros2
  DESTINATION lib/${PROJECT_NAME}
)

# Stereo VIO frontend
add_executable(vio_frontend_stereo_ros2 src/vio_frontend_stereo_ros2.cpp)
ament_target_dependencies(vio_frontend_stereo_ros2 rclcpp sensor_msgs cv_bridge ament_index_cpp video_kit_cameras message_filters geometry_msgs nav_msgs asr_sdm_vio asr_sdm_vio_ceres_backend)
target_link_libraries(vio_frontend_stereo_ros2 cv_bridge::cv_bridge CURL::libcurl ${asr_sdm_vio_LIBRARIES} ${asr_sdm_vio_ceres_backend_LIBRARIES})
# Ensure exported include dirs from asr_sdm_vio are added
if(DEFINED asr_sdm_vio_INCLUDE_DIRS)
  target_include_directories(vio_frontend_ros2 PUBLIC ${asr_sdm_vio_INCLUDE_DIRS})
  target_include_directories(vio_frontend_stereo_ros2 PUBLIC ${asr_sdm_vio_INCLUDE_DIRS})
endif()
# Fallback include hints
if(EXISTS "${CMAKE_INSTALL_PREFIX}/include")
  target_include_directories(vio_frontend_ros2 PUBLIC ${CMAKE_INSTALL_PREFIX}/include)
  target_include_directories(vio_frontend_stereo_ros2 PUBLIC ${CMAKE_INSTALL_PREFIX}/include)
endif()
if(EXISTS "/home/lxy/asr_sdm_ws/install/asr_sdm_vio/include")
  target_include_directories(vio_frontend_ros2 PUBLIC /home/lxy/asr_sdm_ws/install/asr_sdm_vio/include)
  target_include_directories(vio_frontend_stereo_ros2 PUBLIC /home/lxy/asr_sdm_ws/install/asr_sdm_vio/include)
endif()

install(TARGETS vio_frontend_stereo_ros2
  DESTINATION lib/${PROJECT_NAME}
)

# Install resources (launch files, parameters, rviz configs, perspectives, scripts)
install(DIRECTORY
  launch
  param
  DESTINATION share/${PROJECT_NAME}
)

# Optional RVIZ/RQT configs if present
install(FILES
  rviz_config.rviz
  rviz_config_vio.rviz
  rviz_config_gm.rviz
  rqt.perspective
  DESTINATION share/${PROJECT_NAME}
)

# Helper scripts (not entrypoints)
install(DIRECTORY scripts/
  DESTINATION share/${PROJECT_NAME}/scripts
  USE_SOURCE_PERMISSIONS
)

ament_package()
