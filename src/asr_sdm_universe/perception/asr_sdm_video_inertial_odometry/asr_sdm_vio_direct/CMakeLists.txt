cmake_minimum_required(VERSION 3.8)
project(asr_sdm_vio_direct)

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
  include/asr_sdm_vio/direct/patch_warp.h
  include/asr_sdm_vio/direct/patch_score.h
  include/asr_sdm_vio/direct/patch_utils.h
  include/asr_sdm_vio/direct/feature_alignment.h
  include/asr_sdm_vio/direct/feature_detection_types.h
  include/asr_sdm_vio/direct/feature_detection.h
  include/asr_sdm_vio/direct/feature_detection_utils.h
  include/asr_sdm_vio/direct/matcher.h
  include/asr_sdm_vio/direct/depth_filter.h
  include/asr_sdm_vio/direct/depth_estimation.h
  include/asr_sdm_vio/direct/elder_zucker.h
)

set(SOURCES
  src/patch_warp.cpp
  src/feature_alignment.cpp
  src/feature_detection.cpp
  src/feature_detection_utils.cpp
  src/matcher.cpp
  src/depth_filter.cpp
  src/depth_estimation.cpp
  src/elder_zucker.cpp
)

# Create library
add_library(${PROJECT_NAME} SHARED ${SOURCES} ${HEADERS})

# Include directories - map asr_sdm_vio_direct to svo for compatibility
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Link dependencies
target_link_libraries(${PROJECT_NAME} 
  ${OpenCV_LIBRARIES}
  Eigen3::Eigen
  glog::glog
  rclcpp::rclcpp
  ${std_msgs_TARGETS}
  ${cv_bridge_TARGETS}
  asr_sdm_vio_common::asr_sdm_vio_common
  video_kit_ros2::video_kit_ros2
  video_kit_solver::video_kit_solver
  video_kit_common::video_kit_common
)

# Test executables (disabled - need ROS2 migration)
if(FALSE AND asr_sdm_vio_test_utils_FOUND)
  message(STATUS "Building test executables with asr_sdm_vio_test_utils")
  
  add_executable(svo_test_patch_score test/test_patch_score.cpp)
target_include_directories(svo_test_patch_score PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(svo_test_patch_score
  rclcpp
  std_msgs
  cv_bridge
  asr_sdm_vio_common
  asr_sdm_vio_test_utils
  video_kit_ros2
  video_kit_solver
  video_kit_common
  asr_sdm_vio_cmake
)
target_link_libraries(svo_test_patch_score ${PROJECT_NAME} curl)

add_executable(svo_test_edgelet_direction test/test_edgelet_direction.cpp)
target_include_directories(svo_test_edgelet_direction PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(svo_test_edgelet_direction
  rclcpp
  std_msgs
  cv_bridge
  asr_sdm_vio_common
  asr_sdm_vio_test_utils
  video_kit_ros2
  video_kit_solver
  video_kit_common
  asr_sdm_vio_cmake
)
target_link_libraries(svo_test_edgelet_direction ${PROJECT_NAME} curl)

add_executable(svo_test_edgelet_extraction test/test_edgelet_extraction.cpp)
target_include_directories(svo_test_edgelet_extraction PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(svo_test_edgelet_extraction
  rclcpp
  std_msgs
  cv_bridge
  asr_sdm_vio_common
  asr_sdm_vio_test_utils
  video_kit_ros2
  video_kit_solver
  video_kit_common
  asr_sdm_vio_cmake
)
target_link_libraries(svo_test_edgelet_extraction ${PROJECT_NAME} curl)

add_executable(svo_test_feature_alignment test/test_feature_alignment.cpp)
target_include_directories(svo_test_feature_alignment PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(svo_test_feature_alignment
  rclcpp
  std_msgs
  cv_bridge
  asr_sdm_vio_common
  asr_sdm_vio_test_utils
  video_kit_ros2
  video_kit_solver
  video_kit_common
  asr_sdm_vio_cmake
)
target_link_libraries(svo_test_feature_alignment ${PROJECT_NAME} curl)

  # Install test executables
  install(TARGETS
    svo_test_patch_score
    svo_test_edgelet_direction
    svo_test_edgelet_extraction
    svo_test_feature_alignment
    DESTINATION lib/${PROJECT_NAME}
  )
else()
  message(STATUS "Skipping test executables - asr_sdm_vio_test_utils not found")
endif()

################################################################################
###
### GTEST
###

if(BUILD_TESTING)
  # Tests disabled - need ROS2 migration (ros/package.h not available in ROS2)
  message(STATUS "Tests disabled - need ROS2 migration")
  
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

################################################################################
# Install
install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(
  rclcpp
  std_msgs
  cv_bridge
  asr_sdm_vio_common
  video_kit_ros2
  video_kit_solver
  video_kit_common
  asr_sdm_vio_cmake
  Eigen3
  glog
  OpenCV
)

ament_package()
