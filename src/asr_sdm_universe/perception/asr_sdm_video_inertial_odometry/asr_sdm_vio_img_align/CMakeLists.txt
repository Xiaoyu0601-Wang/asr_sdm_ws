cmake_minimum_required(VERSION 3.8)
project(asr_sdm_vio_img_align)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(glog REQUIRED)
find_package(OpenCV REQUIRED)
find_package(asr_sdm_vio_cmake REQUIRED)
find_package(asr_sdm_vio_common REQUIRED)
find_package(asr_sdm_vio_direct REQUIRED)
find_package(video_kit_solver REQUIRED)
find_package(video_kit_ros2 REQUIRED)
find_package(eigen_checks REQUIRED)

include(SvoSetup)

# Override C++ standard to C++17 for ROS2 compatibility
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")

# Add additional warning suppressions for ROS2/modern compiler compatibility
add_compile_options(-Wno-unused-parameter -Wno-vla)

set(HEADERS
  include/asr_sdm_vio_img_align/img_align/sparse_img_align_base.h
  include/asr_sdm_vio_img_align/img_align/sparse_img_align.h
  )

set(SOURCES
  src/sparse_img_align_base.cpp
  src/sparse_img_align.cpp
  )

# Create shared library
add_library(${PROJECT_NAME} SHARED ${SOURCES} ${HEADERS})

# Include directories - map asr_sdm_vio_img_align to svo for compatibility
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIR}
)

# Create a symbolic link for svo includes at configure time
# This allows #include <svo/...> to work with files in include/asr_sdm_vio_img_align/
if(NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/include/svo")
  execute_process(
    COMMAND ${CMAKE_COMMAND} -E create_symlink
      ${CMAKE_CURRENT_SOURCE_DIR}/include/asr_sdm_vio_img_align
      ${CMAKE_CURRENT_SOURCE_DIR}/include/svo
    RESULT_VARIABLE SYMLINK_RESULT
  )
  if(NOT SYMLINK_RESULT EQUAL 0)
    message(WARNING "Failed to create symbolic link for svo includes")
  endif()
endif()

target_link_libraries(${PROJECT_NAME} 
  ${OpenCV_LIBRARIES} 
  glog::glog
  rclcpp::rclcpp
  ${std_msgs_TARGETS}
  asr_sdm_vio_common::asr_sdm_vio_common
  asr_sdm_vio_direct::asr_sdm_vio_direct
  video_kit_solver::video_kit_solver
  video_kit_ros2::video_kit_ros2
)

################################################################################
install(
    TARGETS ${PROJECT_NAME}
    EXPORT export_${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

install(
    DIRECTORY include/
    DESTINATION include
)

ament_export_dependencies(rclcpp std_msgs Eigen3 glog OpenCV asr_sdm_vio_cmake asr_sdm_vio_common asr_sdm_vio_direct video_kit_solver video_kit_ros2)
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
