cmake_minimum_required(VERSION 3.20)
project(asr_sdm_video_enhancement)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_storage REQUIRED)

add_library(asr_sdm_video_enhancement_core src/enhancement_processor.cpp)
add_executable(asr_sdm_video_enhancement src/asr_sdm_video_enhancement.cpp)
add_executable(asr_sdm_mcap_player src/asr_sdm_video_file_publisher.cpp)
add_executable(asr_sdm_video_enhancement_eval src/asr_sdm_video_enhancement_eval.cpp)
add_executable(asr_sdm_video_enhancement_export src/asr_sdm_video_enhancement_export.cpp)

target_include_directories(asr_sdm_video_enhancement_core
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(asr_sdm_video_enhancement_core
  ${OpenCV_LIBRARIES}
)

target_include_directories(asr_sdm_video_enhancement
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${OpenCV_INCLUDE_DIRS}
)

ament_target_dependencies(asr_sdm_video_enhancement
  rclcpp
  sensor_msgs
  std_msgs
  cv_bridge
  image_transport
)

target_link_libraries(asr_sdm_video_enhancement
  asr_sdm_video_enhancement_core
  ${OpenCV_LIBRARIES}
)

target_include_directories(asr_sdm_mcap_player
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(asr_sdm_mcap_player
  rclcpp
  sensor_msgs
  std_msgs
  rosbag2_cpp
  rosbag2_storage
)

target_include_directories(asr_sdm_video_enhancement_eval
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(asr_sdm_video_enhancement_eval
  asr_sdm_video_enhancement_core
  ${OpenCV_LIBRARIES}
)

target_include_directories(asr_sdm_video_enhancement_export
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(asr_sdm_video_enhancement_export
  asr_sdm_video_enhancement_core
  ${OpenCV_LIBRARIES}
)

install(TARGETS
  asr_sdm_video_enhancement_core
  asr_sdm_video_enhancement
  asr_sdm_mcap_player
  asr_sdm_video_enhancement_eval
  asr_sdm_video_enhancement_export
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/convert_mp4_to_mcap
  scripts/annotate_cv1045_demo
  DESTINATION lib/${PROJECT_NAME}
)

install(CODE
  "file(REMOVE \"\$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}/play_video_as_ros_topic.py\")"
)

install(CODE
  "file(REMOVE \"\$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}/asr_sdm_video_file_publisher\")"
)

install(CODE
  "file(REMOVE \"\$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}/asr_sdm_compressed_preview_bridge\")"
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
