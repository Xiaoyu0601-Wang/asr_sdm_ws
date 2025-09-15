cmake_minimum_required(VERSION 3.20)
project(asr_sdm_teleop)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(asr_sdm_control_msgs REQUIRED)

# ament_auto_add_executable(asr_sdm_hardware_node src/asr_sdm_hardware_node.cpp src/uart2can.cpp)
add_executable(asr_sdm_teleop_node src/asr_sdm_teleop_node.cpp)
target_link_libraries(asr_sdm_teleop_node PUBLIC
  ${asr_sdm_control_msgs_TARGETS}
  ${asr_sdm_hardware_msgs_TARGETS}
  ${std_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  rclcpp::rclcpp
)

install(TARGETS
  asr_sdm_teleop_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

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
