cmake_minimum_required(VERSION 3.8)
project(asr_sdm_perception_msgs)

# 默认使用C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# 生成消息接口
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/TrafficLightRoi.msg"
  "msg/TrafficLightRoiArray.msg"
  DEPENDENCIES std_msgs sensor_msgs
)

# 导出依赖
ament_export_dependencies(rosidl_default_runtime)
ament_export_dependencies(std_msgs)
ament_export_dependencies(sensor_msgs)

# 安装
install(
  DIRECTORY msg/
  DESTINATION share/${PROJECT_NAME}/msg
)

ament_package()