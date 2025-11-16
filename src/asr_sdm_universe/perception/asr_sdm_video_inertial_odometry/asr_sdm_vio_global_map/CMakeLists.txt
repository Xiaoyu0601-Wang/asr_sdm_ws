cmake_minimum_required(VERSION 3.8)
project(asr_sdm_vio_global_map)

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
find_package(asr_sdm_vio_imu REQUIRED)
find_package(asr_sdm_rpg_common REQUIRED)
find_package(video_kit_common REQUIRED)

# Find GTSAM
find_package(GTSAMCMakeTools)
find_package(GTSAM REQUIRED)
message(STATUS "GTSAM_INCLUDE_DIR: ${GTSAM_INCLUDE_DIR}")
message(STATUS "GTSAM_LIBRARIES: ${GTSAM_LIBRARIES}")

include(SvoSetup)

# Override C++ standard to C++17 for ROS2 compatibility
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")

set(HEADERS
  # gtsam interface
  include/asr_sdm_vio_global_map/gtsam/gtsam_optimizer.h
  include/asr_sdm_vio_global_map/gtsam/graph_manager.h
  include/asr_sdm_vio_global_map/gtsam/smart_factors_fwd.h
  include/asr_sdm_vio_global_map/gtsam/camera_bearing_factor.h
  include/asr_sdm_vio_global_map/gtsam/camera_bearing_extrinsics_factor.h
  include/asr_sdm_vio_global_map/global_map.h
  include/asr_sdm_vio_global_map/null_stream.h
  )

set(SOURCES
  # gtsam interface
  src/gtsam/gtsam_optimizer.cpp
  src/gtsam/graph_manager.cpp
  src/global_map.cpp
  src/null_stream.cpp
  )

# Create shared library
add_library(${PROJECT_NAME} SHARED ${SOURCES} ${HEADERS})
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIR}
  ${GTSAM_INCLUDE_DIR}
)
target_link_libraries(${PROJECT_NAME} 
  ${OpenCV_LIBRARIES} 
  glog::glog
  gtsam
)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  std_msgs
  Eigen3
  asr_sdm_vio_cmake
  asr_sdm_vio_common
  asr_sdm_vio_imu
  asr_sdm_rpg_common
  video_kit_common
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

ament_export_dependencies(rclcpp std_msgs Eigen3 glog OpenCV asr_sdm_vio_cmake asr_sdm_vio_common asr_sdm_vio_imu asr_sdm_rpg_common video_kit_common)
ament_export_include_directories(include ${GTSAM_INCLUDE_DIR})
ament_export_libraries(${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
