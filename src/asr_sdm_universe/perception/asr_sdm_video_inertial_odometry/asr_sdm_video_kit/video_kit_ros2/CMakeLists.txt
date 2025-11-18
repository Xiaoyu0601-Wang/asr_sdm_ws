cmake_minimum_required(VERSION 3.8)
project(video_kit_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(video_kit_common REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

set(HEADERS
  include/video_kit/img_type_conversion.h
  include/video_kit/output_helper.h
  include/video_kit/params_helper.h
  )

set(SOURCES
  src/output_helper.cpp
  src/img_type_conversion.cpp
  )

add_library(${PROJECT_NAME} SHARED ${SOURCES})
target_link_libraries(${PROJECT_NAME} 
  Eigen3::Eigen
  rclcpp::rclcpp
  ${std_msgs_TARGETS}
  video_kit_common::video_kit_common
  ${sensor_msgs_TARGETS}
  ${cv_bridge_TARGETS}
  ${tf2_ros_TARGETS}
  ${visualization_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

################################################################################
install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)
install(
  DIRECTORY include/
  DESTINATION include
)

ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(rclcpp std_msgs video_kit_common sensor_msgs cv_bridge tf2_ros visualization_msgs geometry_msgs)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
