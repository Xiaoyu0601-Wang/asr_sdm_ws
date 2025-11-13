cmake_minimum_required(VERSION 3.8)
project(asr_sdm_vio_global_map)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(catkin_simple REQUIRED)
catkin_simple()

find_package(GTSAMCMakeTools REQUIRED)
find_package(GTSAM REQUIRED)
message(STATUS ${GTSAM_INCLUDE_DIR})
message(STATUS ${GTSAM_LIBRARIES})

include(SvoSetup)
include_directories(${GTSAM_INCLUDE_DIR})

set(HEADERS
  # gtsam interface
  include/svo/gtsam/gtsam_optimizer.h
  include/svo/gtsam/graph_manager.h
  include/svo/gtsam/smart_factors_fwd.h
  include/svo/gtsam/camera_bearing_factor.h
  include/svo/gtsam/camera_bearing_extrinsics_factor.h
  include/svo/global_map.h
  include/svo/null_stream.h
  )

set(SOURCES
  # gtsam interface
  src/gtsam/gtsam_optimizer.cpp
  src/gtsam/graph_manager.cpp
  src/global_map.cpp
  src/null_stream.cpp
  )

cs_add_library(${PROJECT_NAME} ${SOURCES} ${HEADERS})
target_link_libraries(${PROJECT_NAME} gtsam)

cs_install()
cs_export()

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
