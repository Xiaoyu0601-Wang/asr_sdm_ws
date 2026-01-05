cmake_minimum_required(VERSION 3.8)
project(svo_ros)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-unused-parameter)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(svo REQUIRED)
find_package(asr_sdm_perception_msgs REQUIRED)
find_package(vikit_common REQUIRED)
find_package(vikit_ros REQUIRED)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Sophus REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread)

# Create visualizer library
add_library(svo_visualizer SHARED src/visualizer.cpp)

target_include_directories(svo_visualizer PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Sophus_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

target_link_libraries(svo_visualizer PUBLIC
  ${OpenCV_LIBS}
  Sophus::Sophus
  ${Boost_LIBRARIES}
  svo::svo
)

ament_target_dependencies(svo_visualizer PUBLIC
  rclcpp
  std_msgs
  nav_msgs
  sensor_msgs
  geometry_msgs
  visualization_msgs
  image_transport
  cv_bridge
  tf2
  tf2_ros
  tf2_geometry_msgs
  asr_sdm_perception_msgs
  vikit_common
  vikit_ros
)

# Create Executables
add_executable(vo src/vo_node.cpp)
target_link_libraries(vo svo_visualizer)
ament_target_dependencies(vo
  rclcpp
  sensor_msgs
  std_msgs
  image_transport
  cv_bridge
  vikit_common
  vikit_ros
)

add_executable(vio
  src/vio_node.cpp
  src/msckf_frontend/image_processor.cpp
  src/msckf_frontend/utils.cpp
)
target_include_directories(vio PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(vio ${OpenCV_LIBS} svo::svo)
ament_target_dependencies(vio
  rclcpp
  sensor_msgs
  std_msgs
  geometry_msgs
  image_transport
  cv_bridge
  tf2
  tf2_ros
  tf2_geometry_msgs
  message_filters
  asr_sdm_perception_msgs
)

add_executable(benchmark src/benchmark_node.cpp)
target_link_libraries(benchmark svo_visualizer)
ament_target_dependencies(benchmark
  rclcpp
  sensor_msgs
  image_transport
  cv_bridge
)

# Install
install(TARGETS svo_visualizer
  EXPORT svo_rosTargets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS vo vio benchmark
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch param
  DESTINATION share/${PROJECT_NAME}
)

# Export
ament_export_targets(svo_rosTargets HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_libraries(svo_visualizer)
ament_export_dependencies(
  rclcpp
  std_msgs
  nav_msgs
  sensor_msgs
  geometry_msgs
  visualization_msgs
  image_transport
  cv_bridge
  tf2
  tf2_ros
  tf2_geometry_msgs
  message_filters
  svo
  asr_sdm_perception_msgs
  vikit_common
  vikit_ros
)

ament_package()
