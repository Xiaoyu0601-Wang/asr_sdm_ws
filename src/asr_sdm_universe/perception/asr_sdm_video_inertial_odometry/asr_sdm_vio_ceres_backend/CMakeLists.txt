cmake_minimum_required(VERSION 3.8)
project(asr_sdm_vio_ceres_backend)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(asr_sdm_vio_common REQUIRED)
find_package(asr_sdm_vio_imu REQUIRED)
find_package(video_kit_ros2 REQUIRED)
find_package(Ceres REQUIRED)
find_package(glog REQUIRED)
find_package(asr_sdm_vio REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)
find_package(pcl_conversions REQUIRED)

set(HEADERS
  include/asr_sdm_vio/ceres_backend_interface.hpp
  include/asr_sdm_vio/ceres_backend/estimator.hpp
  include/asr_sdm_vio/ceres_backend/estimator_impl.hpp
  include/asr_sdm_vio/ceres_backend/estimator_types.hpp
  include/asr_sdm_vio/ceres_backend/ceres_iteration_callback.hpp
  include/asr_sdm_vio/ceres_backend/error_interface.hpp
  include/asr_sdm_vio/ceres_backend/homogeneous_point_error.hpp
  include/asr_sdm_vio/ceres_backend/homogeneous_point_local_parameterization.hpp
  include/asr_sdm_vio/ceres_backend/homogeneous_point_parameter_block.hpp
  include/asr_sdm_vio/ceres_backend/imu_error.hpp
  include/asr_sdm_vio/ceres_backend/local_parameterization_additional_interfaces.hpp
  include/asr_sdm_vio/ceres_backend/map.hpp
  include/asr_sdm_vio/ceres_backend/marginalization_error.hpp
  include/asr_sdm_vio/ceres_backend/marginalization_error_impl.hpp
  include/asr_sdm_vio/ceres_backend/parameter_block.hpp
  include/asr_sdm_vio/ceres_backend/pose_error.hpp
  include/asr_sdm_vio/ceres_backend/pose_local_parameterization.hpp
  include/asr_sdm_vio/ceres_backend/pose_parameter_block.hpp
  include/asr_sdm_vio/ceres_backend/relative_pose_error.hpp
  include/asr_sdm_vio/ceres_backend/reprojection_error.hpp
  include/asr_sdm_vio/ceres_backend/reprojection_error_impl.hpp
  include/asr_sdm_vio/ceres_backend/reprojection_error_base.hpp
  include/asr_sdm_vio/ceres_backend/speed_and_bias_error.hpp
  include/asr_sdm_vio/ceres_backend/speed_and_bias_parameter_block.hpp
  include/asr_sdm_vio/ceres_backend/map_alignment.hpp
  include/asr_sdm_vio/ceres_backend/epipolar_error.hpp
  include/asr_sdm_vio/ceres_backend/reprojection_error_simple.hpp
  include/asr_sdm_vio/motion_detector.hpp
  include/asr_sdm_vio/outlier_rejection.hpp
  include/asr_sdm_vio/ceres_backend_publisher.hpp
)

set(SOURCES
  src/ceres_backend_interface.cpp
  src/estimator.cpp
  src/error_interface.cpp
  src/homogeneous_point_error.cpp
  src/homogeneous_point_parameter_block.cpp
  src/homogeneous_point_local_parameterization.cpp
  src/imu_error.cpp
  src/local_parameterization_additional_interfaces.cpp
  src/map.cpp
  src/marginalization_error.cpp
  src/pose_error.cpp
  src/pose_parameter_block.cpp
  src/pose_local_parameterization.cpp
  src/relative_pose_error.cpp
  src/speed_and_bias_error.cpp
  src/speed_and_bias_parameter_block.cpp
  src/motion_detector.cpp
  src/outlier_rejection.cpp
  src/ceres_backend_publisher.cpp
  src/map_alignment.cpp
)

add_library(${PROJECT_NAME} SHARED ${SOURCES} ${HEADERS})

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/asr_sdm_vio>
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  visualization_msgs
  cv_bridge
  tf2_ros
  asr_sdm_vio_common
  asr_sdm_vio_imu
  asr_sdm_vio
  video_kit_ros2
  pcl_conversions
)

target_link_libraries(${PROJECT_NAME}
  Ceres::ceres
  glog::glog
  ${PCL_LIBRARIES}
)

include_directories(${PCL_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  visualization_msgs
  cv_bridge
  tf2_ros
  asr_sdm_vio_common
  asr_sdm_vio_imu
  video_kit_ros2
  Ceres
  glog
)

ament_package()
