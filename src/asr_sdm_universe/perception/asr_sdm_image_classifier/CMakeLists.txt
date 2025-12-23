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
# find_package(asr_sdm_detect_msgs REQUIRED)



# add_executable(img_classifier_node
#   src/img_classifier.cpp
# )


# target_include_directories(img_classifier_node PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>
#   ${OpenCV_INCLUDE_DIRS}
# )


# ament_target_dependencies(img_classifier_node
#   rclcpp
#   sensor_msgs
#   std_msgs
#   cv_bridge
#   asr_sdm_detect_msgs
# )

# target_link_libraries(img_classifier_node
#   ${OpenCV_LIBS}
# )


# install(TARGETS img_classifier_node
#   DESTINATION lib/${PROJECT_NAME}
# )

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
