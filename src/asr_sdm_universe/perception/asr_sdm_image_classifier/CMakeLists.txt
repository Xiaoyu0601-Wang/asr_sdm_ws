cmake_minimum_required(VERSION 3.14)
project(asr_sdm_image_classifier)


set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)   
find_package(std_msgs REQUIRED)      
find_package(cv_bridge REQUIRED)     
find_package(OpenCV REQUIRED COMPONENTS core imgproc dnn highgui)
find_package(asr_sdm_perception_msgs REQUIRED)
find_package(message_filters REQUIRED)


if("$ENV{ROS_DISTRO}" STREQUAL "humble")
  add_definitions(-DROS2_VERSION_HUMBLE)
  message(STATUS "Building for ROS 2 Humble")
elseif("$ENV{ROS_DISTRO}" STREQUAL "jazzy")
  add_definitions(-DROS2_VERSION_JAZZY)
  message(STATUS "Building for ROS 2 Jazzy")
else()
  message(STATUS "Building for unknown ROS version, defaulting to Humble interface")
  add_definitions(-DROS2_VERSION_HUMBLE)
endif()

add_executable(img_classifier_node
  src/img_classifier.cpp
)


target_include_directories(img_classifier_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS}
)


ament_target_dependencies(img_classifier_node
  rclcpp
  sensor_msgs
  std_msgs
  cv_bridge
  asr_sdm_perception_msgs
  message_filters
)

target_link_libraries(img_classifier_node
  ${OpenCV_LIBS}
)


install(TARGETS img_classifier_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
