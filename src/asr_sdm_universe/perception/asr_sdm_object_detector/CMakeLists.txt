cmake_minimum_required(VERSION 3.14)
project(asr_sdm_object_detector)


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




add_executable(img_detector_node
  src/img_detector.cpp
)


target_include_directories(img_detector_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS}          
  /opt/ros/humble/include/        
)


target_link_libraries(img_detector_node
  ${OpenCV_LIBS}
)


if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU" AND CMAKE_CXX_COMPILER_VERSION VERSION_LESS 9.0)
  target_link_libraries(img_detector_node stdc++fs)
endif()


ament_target_dependencies(img_detector_node
  rclcpp
  sensor_msgs
  cv_bridge
  asr_sdm_perception_msgs
)

install(TARGETS img_detector_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
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
