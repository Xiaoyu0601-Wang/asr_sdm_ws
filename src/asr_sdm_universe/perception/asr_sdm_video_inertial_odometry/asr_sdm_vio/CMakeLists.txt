cmake_minimum_required(VERSION 3.8)
project(asr_sdm_vio)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(asr_sdm_vio_cmake REQUIRED)
find_package(asr_sdm_vio_common REQUIRED)
find_package(asr_sdm_vio_direct REQUIRED)
find_package(asr_sdm_vio_tracker REQUIRED)
find_package(asr_sdm_vio_test_utils REQUIRED)
find_package(asr_sdm_vio_img_align REQUIRED)
find_package(asr_sdm_vio_online_loopclosing REQUIRED)


find_package(asr_sdm_rpg_common REQUIRED)
find_package(video_kit_common REQUIRED)
find_package(video_kit_cameras REQUIRED)
find_package(video_kit_solver REQUIRED)
find_package(video_kit_ros2 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(minkindr REQUIRED)
find_package(fast REQUIRED)
find_package(opengv REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(glog REQUIRED)
find_package(asr_sdm_vio_imu REQUIRED)


# user build settings
SET(USE_LOOP_CLOSING TRUE)
SET(USE_GLOBAL_MAP FALSE)

# Set definitions
IF(USE_LOOP_CLOSING)
  add_definitions(-DSVO_LOOP_CLOSING)
ENDIF()

IF(USE_GLOBAL_MAP)
  add_definitions(-DSVO_GLOBAL_MAP)
ENDIF()

add_definitions(-DSVO_USE_ROS)
add_definitions(-DSVO_USE_OPENGV)
add_definitions(-DSVO_DEPTHFILTER_IN_REPROJECTOR)

#############################################################################
# Set build flags, set ARM_ARCHITECTURE environment variable on Odroid
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -Wall -Werror -D_LINUX -D_REENTRANT -march=native -Wno-unused-variable -Wno-unused-but-set-variable -Wno-unknown-pragmas -Wno-unused-but-set-parameter -Wno-int-in-bool-context -Wno-maybe-uninitialized -Wno-unused-function")

IF(DEFINED ENV{ARM_ARCHITECTURE})
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mfpu=neon -march=armv7-a")
  add_definitions(-DHAVE_FAST_NEON)
ELSE()
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mmmx -msse -msse2 -msse3 -mssse3 -mno-avx")
ENDIF()
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3 -fsee -fomit-frame-pointer -fno-signed-zeros -fno-math-errno -funroll-loops -ffast-math -fno-finite-math-only")

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
  include/asr_sdm_vio/abstract_bundle_adjustment.h
  include/asr_sdm_vio/frame_handler_base.h
  include/asr_sdm_vio/frame_handler_mono.h
  include/asr_sdm_vio/frame_handler_stereo.h
  include/asr_sdm_vio/frame_handler_array.h
  include/asr_sdm_vio/global.h
  include/asr_sdm_vio/imu_handler.h
  include/asr_sdm_vio/initialization.h
  include/asr_sdm_vio/map.h
  include/asr_sdm_vio/pose_optimizer.h
  include/asr_sdm_vio/reprojector.h
  include/asr_sdm_vio/stereo_triangulation.h
  include/asr_sdm_vio/io.h
  include/asr_sdm_vio/svo.h
  )

################################################################################
# Create svo library
include_directories(include)

add_library(svo SHARED ${SVO_SOURCEFILES})
target_include_directories(svo PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(svo
  rclcpp::rclcpp
  ${std_msgs_TARGETS}
  asr_sdm_vio_common::asr_sdm_vio_common
  asr_sdm_vio_direct::asr_sdm_vio_direct
  asr_sdm_vio_tracker::asr_sdm_vio_tracker
  asr_sdm_vio_test_utils::asr_sdm_vio_test_utils
  asr_sdm_vio_img_align::asr_sdm_vio_img_align
  asr_sdm_vio_online_loopclosing::asr_sdm_vio_online_loopclosing
  asr_sdm_vio_imu::asr_sdm_vio_imu

  asr_sdm_rpg_common::asr_sdm_rpg_common
  video_kit_common::video_kit_common
  video_kit_cameras::video_kit_cameras
  video_kit_solver::video_kit_solver
  video_kit_ros2::video_kit_ros2
  Eigen3::Eigen
  minkindr::minkindr
  fast::fast
  opengv::opengv
  cv_bridge::cv_bridge
  glog::glog
)

################################################################################
###
### GTEST
###

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(eigen_checks REQUIRED)
  ament_add_gtest(${PROJECT_NAME}-test-frame test/test_frame.cpp)
  target_link_libraries(${PROJECT_NAME}-test-frame svo eigen_checks::eigen_checks asr_sdm_vio_test_utils::asr_sdm_vio_test_utils)
endif()

################################################################################
# Install
install(TARGETS svo
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)
ament_export_libraries(svo)
ament_export_dependencies(
  rclcpp
  std_msgs
  asr_sdm_vio_common
  asr_sdm_vio_direct
  asr_sdm_vio_tracker
  asr_sdm_vio_test_utils
  asr_sdm_vio_img_align
  asr_sdm_vio_online_loopclosing
  asr_sdm_vio_imu
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
