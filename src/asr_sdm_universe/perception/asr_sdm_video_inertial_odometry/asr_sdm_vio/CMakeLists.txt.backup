cmake_minimum_required(VERSION 3.8)
project(asr_sdm_vio)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(catkin_simple)
catkin_simple()

include(SvoSetup)

list(APPEND SVO_SOURCEFILES
  src/frame_handler_mono.cpp
  src/frame_handler_stereo.cpp
  src/frame_handler_array.cpp
  src/frame_handler_base.cpp
  src/map.cpp
  src/pose_optimizer.cpp
  src/initialization.cpp
  src/reprojector.cpp
  src/imu_handler.cpp
  src/stereo_triangulation.cpp
  src/io.cpp
)

list(APPEND SVO_HEADERFILES
  include/svo/abstract_bundle_adjustment.h
  include/svo/frame_handler_base.h
  include/svo/frame_handler_mono.h
  include/svo/frame_handler_stereo.h
  include/svo/frame_handler_array.h
  include/svo/global.h
  include/svo/imu_handler.h
  include/svo/initialization.h
  include/svo/map.h
  include/svo/pose_optimizer.h
  include/svo/reprojector.h
  include/svo/stereo_triangulation.h
  include/svo/io.h
  include/svo/svo.h
  )

################################################################################
# Create svo library
include_directories(include ${INCLUDE_DIRS})
cs_add_library(svo SHARED ${SVO_SOURCEFILES} ${SVO_HEADERFILES})
target_link_libraries(svo ${LINK_LIBS})

################################################################################
###
### GTEST
###

catkin_add_gtest(${PROJECT_NAME}-test-frame test/test_frame.cpp)
target_link_libraries(${PROJECT_NAME}-test-frame ${PROJECT_NAME})

################################################################################
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
