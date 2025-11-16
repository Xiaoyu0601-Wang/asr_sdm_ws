cmake_minimum_required(VERSION 3.8)
project(asr_sdm_vio_common)

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
find_package(video_kit_common REQUIRED)
find_package(video_kit_cameras REQUIRED)
find_package(fast REQUIRED)
find_package(eigen_checks REQUIRED)

include(SvoSetup)

# Override C++ standard to C++17 for ROS2 compatibility
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")

set(HEADERS
  include/asr_sdm_vio_common/common/camera.h
  include/asr_sdm_vio_common/common/camera_fwd.h
  include/asr_sdm_vio_common/common/container_helpers.h
  include/asr_sdm_vio_common/common/conversions.h
  include/asr_sdm_vio_common/common/feature_wrapper.h
  include/asr_sdm_vio_common/common/frame.h
  include/asr_sdm_vio_common/common/imu_calibration.h
  include/asr_sdm_vio_common/common/logging.h
  include/asr_sdm_vio_common/common/occupancy_grid_2d.h
  include/asr_sdm_vio_common/common/point.h
  include/asr_sdm_vio_common/common/seed.h
  include/asr_sdm_vio_common/common/transformation.h
  include/asr_sdm_vio_common/common/types.h
  )

set(SOURCES
  src/frame.cpp
  src/point.cpp
  )

# Create a symbolic link for svo includes at configure time
# This allows #include <svo/...> to work with files in include/asr_sdm_vio_common/
if(NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/include/svo")
  execute_process(
    COMMAND ${CMAKE_COMMAND} -E create_symlink
      ${CMAKE_CURRENT_SOURCE_DIR}/include/asr_sdm_vio_common
      ${CMAKE_CURRENT_SOURCE_DIR}/include/svo
    RESULT_VARIABLE SYMLINK_RESULT
  )
  if(NOT SYMLINK_RESULT EQUAL 0)
    message(WARNING "Failed to create symbolic link for svo includes")
  endif()
endif()

# Create a symbolic link for svo includes at configure time
# This allows #include <svo/...> to work with files in include/asr_sdm_vio_common/
if(NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/include/svo")
  execute_process(
    COMMAND ${CMAKE_COMMAND} -E create_symlink
      ${CMAKE_CURRENT_SOURCE_DIR}/include/asr_sdm_vio_common
      ${CMAKE_CURRENT_SOURCE_DIR}/include/svo
    RESULT_VARIABLE SYMLINK_RESULT
  )
  if(NOT SYMLINK_RESULT EQUAL 0)
    message(WARNING "Failed to create symbolic link for svo includes")
  endif()
endif()

# Create a symbolic link for svo includes at configure time
# This allows #include <svo/...> to work with files in include/asr_sdm_vio_common/
if(NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/include/svo")
  execute_process(
    COMMAND ${CMAKE_COMMAND} -E create_symlink
      ${CMAKE_CURRENT_SOURCE_DIR}/include/asr_sdm_vio_common
      ${CMAKE_CURRENT_SOURCE_DIR}/include/svo
    RESULT_VARIABLE SYMLINK_RESULT
  )
  if(NOT SYMLINK_RESULT EQUAL 0)
    message(WARNING "Failed to create symbolic link for svo includes")
  endif()
endif()

# Create shared library
add_library(${PROJECT_NAME} SHARED ${SOURCES} ${HEADERS})
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIR}
)
target_link_libraries(${PROJECT_NAME} 
  ${OpenCV_LIBRARIES} 
  glog::glog
  rclcpp::rclcpp
  ${std_msgs_TARGETS}
  video_kit_common::video_kit_common
  video_kit_cameras::video_kit_cameras
  fast::fast
)

################################################################################
# Unit tests
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  
  ament_add_gtest(${PROJECT_NAME}-test
    test/test_main.cpp
    test/test_feature_wrapper.cpp
  )
  target_link_libraries(${PROJECT_NAME}-test 
    ${PROJECT_NAME}
    eigen_checks::eigen_checks
  )

  ament_add_gtest(container-helpers-test
    test/test_container_helpers.cpp
  )
  target_link_libraries(container-helpers-test ${PROJECT_NAME})
endif()

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

ament_export_dependencies(rclcpp std_msgs Eigen3 glog OpenCV asr_sdm_vio_cmake video_kit_common video_kit_cameras fast)
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
