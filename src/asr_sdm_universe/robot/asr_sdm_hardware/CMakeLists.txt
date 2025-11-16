cmake_minimum_required(VERSION 3.8)
project(asr_sdm_hardware)

# Default to C++17
# if(NOT CMAKE_CXX_STANDARD)
#   set(CMAKE_CXX_STANDARD 17)
# endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(asr_sdm_control_msgs REQUIRED)
find_package(asr_sdm_hardware_msgs REQUIRED)
find_package(asr_sdm_imu_hiwonder_10axis REQUIRED)
find_package(data_structure REQUIRED)
find_package(periphery REQUIRED)

include_directories(
  include
  /usr/local/include
  ${asr_sdm_imu_hiwonder_10axis_INCLUDE_DIRS}
  ${data_structure_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME} src/asr_sdm_hardware_node.cpp src/uart2can.cpp src/comm_protocol.cpp)
target_link_libraries(${PROJECT_NAME} PUBLIC
  rclcpp::rclcpp
  ${std_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${asr_sdm_control_msgs_TARGETS}
  ${asr_sdm_hardware_msgs_TARGETS}
  ${asr_sdm_imu_hiwonder_10axis_TARGETS}  # asr_sdm_imu_hiwonder_10axis
  data_structure::data_structure
  periphery
)

install(TARGETS
  ${PROJECT_NAME}
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

ament_export_dependencies(rosidl_default_runtime)
ament_package()
