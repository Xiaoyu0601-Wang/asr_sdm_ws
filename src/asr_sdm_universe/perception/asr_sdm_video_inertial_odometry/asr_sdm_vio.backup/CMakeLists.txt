cmake_minimum_required(VERSION 3.8)
project(asr_sdm_vio)

# Default C++ standard
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)

# ROS 2 messages (initial step)
set(MSG_FILES
  "msg/FeatureMeasurement.msg"
  "msg/CameraMeasurement.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${MSG_FILES}
  DEPENDENCIES builtin_interfaces std_msgs geometry_msgs sensor_msgs
)

# Node (ROS2 skeleton) using generated messages
add_executable(msckf_vio_node src/ros2/msckf_vio_node.cpp)
ament_target_dependencies(msckf_vio_node rclcpp sensor_msgs nav_msgs)
# Link generated message interfaces
rosidl_target_interfaces(msckf_vio_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

install(TARGETS msckf_vio_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install public headers (for future C++ targets when porting completes)
install(DIRECTORY include/
  DESTINATION include
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()

