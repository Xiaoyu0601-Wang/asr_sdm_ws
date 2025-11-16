cmake_minimum_required(VERSION 3.8)
project(asr_sdm_vio_test_utils)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(asr_sdm_vio_common REQUIRED)
find_package(video_kit_ros2 REQUIRED)
find_package(video_kit_common REQUIRED)
find_package(video_kit_cameras REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(glog REQUIRED)

################################################################################
set(HEADERS
  include/asr_sdm_vio_test_utils/test_utils/test_utils.h
  include/asr_sdm_vio_test_utils/test_utils/imu_simulation.h
  include/asr_sdm_vio_test_utils/test_utils/synthetic_dataset.h
  include/asr_sdm_vio_test_utils/test_utils/simple_raytracer.h
  include/asr_sdm_vio_test_utils/test_utils/trajectory_generator.h
  include/asr_sdm_vio_test_utils/test_utils/point_cloud_generator.h
)

set(SOURCES
  src/test_utils.cpp
  src/imu_simulation.cpp
  src/synthetic_dataset.cpp
  src/simple_raytracer.cpp
  src/trajectory_generator.cpp
  src/point_cloud_generator.cpp
)

# Create a symbolic link for svo includes at configure time
# This allows #include <svo/...> to work with files in include/asr_sdm_vio_test_utils/
if(NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/include/svo")
  execute_process(
    COMMAND ${CMAKE_COMMAND} -E create_symlink
      ${CMAKE_CURRENT_SOURCE_DIR}/include/asr_sdm_vio_test_utils
      ${CMAKE_CURRENT_SOURCE_DIR}/include/svo
    RESULT_VARIABLE SYMLINK_RESULT
  )
  if(NOT SYMLINK_RESULT EQUAL 0)
    message(WARNING "Failed to create symbolic link for svo includes")
  endif()
endif()

# Create library
add_library(${PROJECT_NAME} SHARED ${SOURCES} ${HEADERS})

# Include directories
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
)

# Link dependencies
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  std_msgs
  cv_bridge
  asr_sdm_vio_common
  video_kit_ros2
  video_kit_common
  video_kit_cameras
  OpenCV
)

target_link_libraries(${PROJECT_NAME}
  glog::glog
  Eigen3::Eigen
)

# Install library
install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}Targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install headers
install(DIRECTORY include/
  DESTINATION include
)

# Export targets
ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp
  std_msgs
  cv_bridge
  asr_sdm_vio_common
  video_kit_ros2
  video_kit_common
  video_kit_cameras
  OpenCV
  Eigen3
  glog
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

ament_package()
