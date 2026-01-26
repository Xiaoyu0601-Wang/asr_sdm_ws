cmake_minimum_required(VERSION 3.20)
project(asr_sdm_underwater_video_enhancement)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# =====================================================================
# Dependencies
# =====================================================================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)


# =====================================================================
# Header detection (IMPORTANT)
# =====================================================================
include(CheckIncludeFileCXX)
check_include_file_cxx(cv_bridge/cv_bridge.hpp HAVE_CV_BRIDGE_HPP)


# =====================================================================
# Target
# =====================================================================
add_executable(asr_sdm_underwater_video_enhancement src/asr_sdm_underwater_video_enhancement.cpp)

target_include_directories(asr_sdm_underwater_video_enhancement
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${cv_bridge_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(asr_sdm_underwater_video_enhancement PUBLIC
  rclcpp::rclcpp
  ${sensor_msgs_TARGETS}
  ${cv_bridge_TARGETS}
  ${OpenCV_LIBRARIES}
)

# =====================================================================
# Compile definitions (bind to target!)
# =====================================================================
if(HAVE_CV_BRIDGE_HPP)
  add_compile_definitions(HAVE_CV_BRIDGE_HPP)
endif()

# =====================================================================
# Install
# =====================================================================
install(TARGETS
  asr_sdm_underwater_video_enhancement
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Install config files
install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

# Install scripts
install(
  DIRECTORY scripts
  DESTINATION share/${PROJECT_NAME}
  USE_SOURCE_PERMISSIONS
)

# =====================================================================
# Testing
# =====================================================================
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
