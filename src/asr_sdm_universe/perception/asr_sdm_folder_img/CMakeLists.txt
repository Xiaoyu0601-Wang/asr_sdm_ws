cmake_minimum_required(VERSION 3.8)
project(asr_sdm_folder_publisher)

set(CMAKE_CXX_STANDARD 17)

# 查找依赖包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

# 添加可执行文件
add_executable(image_folder_publisher
  src/image_folder_publisher.cpp
)

# 使用ament_target_dependencies，这会自动处理链接
ament_target_dependencies(image_folder_publisher
  rclcpp
  sensor_msgs
  cv_bridge
)

# 单独链接OpenCV
target_link_libraries(image_folder_publisher
  ${OpenCV_LIBS}
)

# 安装可执行文件
install(TARGETS image_folder_publisher
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# 安装启动文件
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()