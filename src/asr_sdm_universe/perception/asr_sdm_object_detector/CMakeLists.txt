cmake_minimum_required(VERSION 3.14)
project(asr_sdm_object_detector)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# 查找所有必需依赖（ROS2 + OpenCV）
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED) 
find_package(sensor_msgs REQUIRED)       
find_package(cv_bridge REQUIRED)          
find_package(vision_msgs REQUIRED)        
find_package(OpenCV REQUIRED COMPONENTS core imgproc dnn highgui) 

# 编译源文件
add_library(${PROJECT_NAME} SHARED
  src/img_detector.cpp
)

# 头文件路径
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS}  # OpenCV头文件
  /opt/ros/humble/include/ 
)

target_link_libraries(${PROJECT_NAME}
  ${OpenCV_LIBS}  # OpenCV 库
  rclcpp_components::component 
)

# GCC < 9 兼容 filesystem（旧系统需要）
if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU" AND CMAKE_CXX_COMPILER_VERSION VERSION_LESS 9.0)
  target_link_libraries(${PROJECT_NAME} stdc++fs)
endif()

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  sensor_msgs
  cv_bridge
  vision_msgs
)

# 注册节点
rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "img_detector::ImgDetectorNode"
  EXECUTABLE img_detector_node
)

# 安装库和可执行文件
install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# 安装Launch和Config文件夹
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
)

# 测试
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()