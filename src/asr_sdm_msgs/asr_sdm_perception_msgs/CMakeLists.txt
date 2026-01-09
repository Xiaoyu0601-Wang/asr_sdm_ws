cmake_minimum_required(VERSION 3.8)
project(asr_sdm_perception_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/TrafficLightRoi.msg"
  "msg/TrafficLightRoiArray.msg"
  "msg/DenseInput.msg"
  "msg/Feature.msg"
  "msg/Info.msg"
  "msg/NbvTrajectory.msg"
  "msg/FeatureMeasurement.msg"
  "msg/CameraMeasurement.msg"
  "msg/TrackingInfo.msg"
  DEPENDENCIES std_msgs sensor_msgs geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
