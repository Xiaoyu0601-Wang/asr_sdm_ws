cmake_minimum_required(VERSION 3.8)
project(svo)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-unused-parameter)
endif()

# User build settings
option(TRACE "Enable tracing" ON)
option(HAVE_G2O "Use G2O bundle adjustment" OFF)
option(DEBUG_OUTPUT "Enable debug output" OFF)

# Set definitions
if(TRACE)
  add_definitions(-DSVO_TRACE)
endif()
if(HAVE_G2O)
  add_definitions(-DUSE_BUNDLE_ADJUSTMENT)
endif()
if(DEBUG_OUTPUT)
  add_definitions(-DSVO_DEBUG_OUTPUT)
endif()

# Always use ROS2
add_definitions(-DSVO_USE_ROS)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Sophus REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread system)
find_package(vikit_common REQUIRED)
find_package(vikit_ros REQUIRED)
find_package(fast REQUIRED)

# Set sourcefiles
set(SOURCEFILES
  src/frame_handler_mono.cpp
  src/frame_handler_base.cpp
  src/frame.cpp
  src/point.cpp
  src/map.cpp
  src/pose_optimizer.cpp
  src/initialization.cpp
  src/matcher.cpp
  src/reprojector.cpp
  src/feature_alignment.cpp
  src/feature_detection.cpp
  src/depth_filter.cpp
  src/config.cpp
  src/sparse_img_align.cpp
)

# Add g2o if available
if(HAVE_G2O)
  find_package(G2O REQUIRED)
  list(APPEND SOURCEFILES src/bundle_adjustment.cpp)
endif()

# Create svo library
add_library(svo SHARED ${SOURCEFILES})

target_include_directories(svo PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Sophus_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

target_link_libraries(svo PUBLIC
  ${OpenCV_LIBS}
  Sophus::Sophus
  fast::fast
)

target_link_libraries(svo PRIVATE
  ${Boost_LIBRARIES}
)

ament_target_dependencies(svo PUBLIC
  rclcpp
  vikit_common
  vikit_ros
)

if(HAVE_G2O)
  target_include_directories(svo PUBLIC ${G2O_INCLUDE_DIR})
  target_link_libraries(svo PUBLIC
    ${G2O_CORE_LIBRARY}
    ${G2O_STUFF_LIBRARY}
    ${G2O_SOLVER_CHOLMOD}
    ${G2O_SOLVER_CSPARSE}
    ${G2O_SOLVER_DENSE}
    ${G2O_SOLVER_PCG}
    ${G2O_TYPES_SBA}
    cholmod
    cxsparse
  )
endif()

# Tests disabled for now - need ROS2 porting
add_executable(test_feature_align test/test_feature_alignment.cpp)
target_link_libraries(test_feature_align svo)
ament_target_dependencies(test_feature_align ament_index_cpp)
# 
add_executable(test_pipeline test/test_pipeline.cpp)
target_link_libraries(test_pipeline svo)
ament_target_dependencies(test_pipeline ament_index_cpp)

add_executable(test_pipeline_euroc test/test_pipeline_euroc.cpp)
target_link_libraries(test_pipeline_euroc svo)
ament_target_dependencies(test_pipeline_euroc ament_index_cpp)
# 
add_executable(test_matcher test/test_matcher.cpp)
target_link_libraries(test_matcher svo)
ament_target_dependencies(test_matcher ament_index_cpp)
# 
add_executable(test_feature_detection test/test_feature_detection.cpp)
target_link_libraries(test_feature_detection svo)
ament_target_dependencies(test_feature_detection ament_index_cpp)
# 
add_executable(test_depth_filter test/test_depth_filter.cpp)
target_link_libraries(test_depth_filter svo)
ament_target_dependencies(test_depth_filter ament_index_cpp)
# 
add_executable(test_sparse_img_align test/test_sparse_img_align.cpp)
target_link_libraries(test_sparse_img_align svo)
ament_target_dependencies(test_sparse_img_align ament_index_cpp)
# 
add_executable(test_pose_optimizer test/test_pose_optimizer.cpp)
target_link_libraries(test_pose_optimizer svo)
ament_target_dependencies(test_pose_optimizer ament_index_cpp)

# Install
install(TARGETS svo
  EXPORT svoTargets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(TARGETS
  test_depth_filter
  test_feature_align
  test_feature_detection
  test_matcher
  test_pipeline
  test_pipeline_euroc
  test_pose_optimizer
  test_sparse_img_align
  DESTINATION lib/${PROJECT_NAME}
)

# Export
ament_export_targets(svoTargets HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_libraries(svo)
ament_export_dependencies(
  rclcpp
  OpenCV
  Eigen3
  Sophus
  vikit_common
  vikit_ros
  fast
)

ament_package()
