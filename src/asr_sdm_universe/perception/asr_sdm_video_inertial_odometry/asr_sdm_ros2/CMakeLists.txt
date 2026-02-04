cmake_minimum_required(VERSION 3.8)
project(asr_sdm_ros2)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)

find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(asr_sdm_msgs REQUIRED)
find_package(asr_sdm_rpg_common REQUIRED)
find_package(OpenCV REQUIRED)
find_package(CURL REQUIRED)
find_package(glog REQUIRED)

find_package(vikit_cameras REQUIRED)
find_package(asr_sdm_svo2 REQUIRED)
find_package(asr_sdm_ceres_backend REQUIRED)
find_package(image_transport REQUIRED)
find_package(yaml-cpp REQUIRED)

set(USE_GTSAM false)

set(HEADERS
  include/asr_sdm_ros2/svo_interface.h
  include/asr_sdm_ros2/svo_factory.h
  include/asr_sdm_ros2/visualizer.h
  include/asr_sdm_ros2/svo_nodelet.h
  include/asr_sdm_ros2/csv_dataset_reader.h
  include/asr_sdm_ros2/ceres_backend_factory.h
  include/asr_sdm_ros2/svo_node_base.h
)

set(SOURCES
  src/ceres_backend_factory.cpp
  src/svo_interface.cpp
  src/svo_factory.cpp
  src/svo_node_base.cpp
  src/visualizer.cpp
  src/csv_dataset_reader.cpp
)

# Optional GTSAM backend
if(USE_GTSAM)
  find_package(GTSAM REQUIRED)
  list(APPEND HEADERS
    include/asr_sdm_ros2/backend_visualizer.h
    include/asr_sdm_ros2/backend_factory.h)
  list(APPEND SOURCES
    src/backend_visualizer.cpp
    src/backend_factory.cpp)
endif()

add_library(${PROJECT_NAME} ${SOURCES} ${HEADERS})
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rclcpp_components
  sensor_msgs
  std_msgs
  geometry_msgs
  nav_msgs
  tf2_ros
  tf2
  tf2_geometry_msgs
  visualization_msgs
  message_filters
  cv_bridge
  pcl_conversions
  image_transport
  vikit_cameras
  asr_sdm_msgs
  asr_sdm_rpg_common
  asr_sdm_svo2
  asr_sdm_ceres_backend
  asr_sdm_online_loopclosing
)

target_link_libraries(${PROJECT_NAME}
  ${OpenCV_LIBRARIES}
  CURL::libcurl
  glog::glog
  yaml-cpp
)

# Add SVO compile definitions to ensure consistent ABI across all linked libraries
target_compile_definitions(${PROJECT_NAME} PRIVATE
  -DSVO_LOOP_CLOSING
  -DSVO_USE_OPENGV
  -DSVO_USE_OPENGV_OMNIDIRECTIONAL
  -DSVO_USE_OPENGV_OMNIDIRECTIONAL_RANSAC
  -DSVO_USE_OPENGV_OMNIDIRECTIONAL_NISTER
  -DSVO_USE_OPENGV_OMNIDIRECTIONAL_STEWENIUS
  -DSVO_USE_OPENGV_OMNIDIRECTIONAL_LI
  -DSVO_USE_OPENGV_OMNIDIRECTIONAL_LI_RANSAC
  -DSVO_USE_OPENGV_OMNIDIRECTIONAL_LI_NISTER
  -DSVO_USE_OPENGV_OMNIDIRECTIONAL_LI_STEWENIUS
)

if(USE_GTSAM)
  target_compile_definitions(${PROJECT_NAME} PRIVATE -DSVO_USE_GTSAM_BACKEND)
  target_link_libraries(${PROJECT_NAME} gtsam)
endif()


add_executable(svo_benchmark src/benchmark_node.cpp)
ament_target_dependencies(svo_benchmark rclcpp)
target_link_libraries(svo_benchmark ${PROJECT_NAME} asan)

add_executable(svo_node src/svo_node.cpp)
ament_target_dependencies(svo_node rclcpp)
target_link_libraries(svo_node ${PROJECT_NAME} asan)

# ROS2 Component (Nodelet equivalent)
add_library(asr_sdm_ros2_nodelet SHARED src/svo_nodelet.cpp)
target_link_libraries(asr_sdm_ros2_nodelet ${PROJECT_NAME})
ament_target_dependencies(asr_sdm_ros2_nodelet rclcpp rclcpp_components)

rclcpp_components_register_nodes(asr_sdm_ros2_nodelet "svo::SvoNodelet")

install(TARGETS
  ${PROJECT_NAME}
  svo_benchmark
  svo_node
  asr_sdm_ros2_nodelet
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY
  launch
  param
  DESTINATION share/${PROJECT_NAME}
)

install(FILES
  rviz_config.rviz
  rviz_config_vio.rviz
  rviz_config_gm.rviz
  rqt.perspective
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
