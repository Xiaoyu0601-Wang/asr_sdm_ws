cmake_minimum_required(VERSION 3.20)
project(asr_sdm_imu_hiwonder_10axis)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(data_structure REQUIRED)
find_package(periphery REQUIRED)

include_directories(
  include
  /usr/local/include
)

# # Declare a header-only "library"
add_library(data_structure INTERFACE)

add_executable(asr_sdm_imu_hiwonder_10axis_node src/asr_sdm_imu_hiwonder_10axis_node.cpp)
target_include_directories(asr_sdm_imu_hiwonder_10axis_node PUBLIC
  $<BUILD_INTERFACE:${data_structure_INCLUDE_DIRS}>
  $<INSTALL_INTERFACE:${data_structure_INCLUDE_DIRS}>
)
target_link_libraries(asr_sdm_imu_hiwonder_10axis_node PUBLIC
  ${std_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  data_structure
  rclcpp::rclcpp
  periphery
)

install(TARGETS
  asr_sdm_imu_hiwonder_10axis_node
  DESTINATION lib/${PROJECT_NAME}
)

# install(
#   DIRECTORY config
#   DESTINATION share/${PROJECT_NAME}
# )

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
