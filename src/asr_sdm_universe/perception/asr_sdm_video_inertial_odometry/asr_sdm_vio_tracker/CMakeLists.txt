cmake_minimum_required(VERSION 3.8)
project(asr_sdm_vio_tracker)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(glog REQUIRED)
find_package(asr_sdm_vio_common REQUIRED)
find_package(asr_sdm_vio_direct REQUIRED)
find_package(asr_sdm_vio_test_utils QUIET)
find_package(video_kit_ros2 REQUIRED)
find_package(video_kit_solver REQUIRED)
find_package(video_kit_common REQUIRED)
find_package(asr_sdm_vio_cmake REQUIRED)
find_package(OpenCV REQUIRED)

# Include SvoSetup
include(SvoSetup)

# Override C++ standard to C++17 for ROS2 compatibility
# set(CMAKE_CXX_STANDARD 17)
# set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Add additional warning suppressions for ROS2/modern compiler compatibility
add_compile_options(-Wno-unused-parameter -Wno-vla)

set(HEADERS
  include/asr_sdm_vio/tracker/feature_tracking_types.h
  include/asr_sdm_vio/tracker/feature_tracking_utils.h
  include/asr_sdm_vio/tracker/feature_tracking_viz.h
  include/asr_sdm_vio/tracker/feature_tracker.h
)

set(SOURCES
  src/feature_tracking_types.cpp
  src/feature_tracking_utils.cpp
  src/feature_tracking_viz.cpp
  src/feature_tracker.cpp
)

# Create library
add_library(${PROJECT_NAME} SHARED ${SOURCES})

# Include directories - map asr_sdm_vio_tracker to svo for compatibility
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)



# Link dependencies
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  std_msgs
  cv_bridge
  asr_sdm_vio_common
  asr_sdm_vio_direct
  video_kit_ros2
  video_kit_solver
  video_kit_common
  asr_sdm_vio_cmake
)
target_link_libraries(${PROJECT_NAME} 
  ${OpenCV_LIBRARIES}
  Eigen3::Eigen
  glog::glog
)

################################################################################

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

################################################################################
# Install
install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(
  rclcpp
  std_msgs
  cv_bridge
  asr_sdm_vio_common
  asr_sdm_vio_direct
  video_kit_ros2
  video_kit_solver
  video_kit_common
  asr_sdm_vio_cmake
  Eigen3
  glog
)

ament_package()
