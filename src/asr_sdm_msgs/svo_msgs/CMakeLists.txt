cmake_minimum_required(VERSION 3.8)
project(svo_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# Generate messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/DenseInput.msg"
  "msg/Feature.msg"
  "msg/Info.msg"
  "msg/NbvTrajectory.msg"
  "msg/FeatureMeasurement.msg"
  "msg/CameraMeasurement.msg"
  "msg/TrackingInfo.msg"
  DEPENDENCIES geometry_msgs sensor_msgs std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_export_dependencies(geometry_msgs)
ament_export_dependencies(sensor_msgs)
ament_export_dependencies(std_msgs)

ament_package()