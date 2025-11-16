cmake_minimum_required(VERSION 3.8)
project(video_kit_common)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(glog REQUIRED)
find_package(minkindr REQUIRED)
find_package(asr_sdm_vio_cmake REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(OpenCV REQUIRED)

include(SvoSetup)

set(HEADERS
  include/video_kit/blender_utils.h
  include/video_kit/math_utils.h
  include/video_kit/performance_monitor.h
  include/video_kit/ringbuffer.h
  include/video_kit/sample.h
  include/video_kit/timer.h
  include/video_kit/user_input_thread.h
  include/video_kit/vision.h
  include/video_kit/homography_decomp.h
  include/video_kit/homography.h
  include/video_kit/path_utils.h
  include/video_kit/csv_utils.h
  include/aslam/common/entrypoint.h
  include/aslam/common/macros.h
  include/aslam/common/memory.h
  include/aslam/common/pose-types.h
  include/aslam/common/yaml-serialization.h
  include/aslam/common/yaml-serialization-eigen.h
  )

set(SOURCES
  src/homography.cpp
  src/math_utils.cpp
  src/performance_monitor.cpp
  src/sample.cpp
  src/user_input_thread.cpp
  src/vision.cpp
  )

# Create vikit shared library
add_library(${PROJECT_NAME} SHARED ${SOURCES} ${HEADERS})
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIR}
  ${minkindr_INCLUDE_DIRS}
)
target_link_libraries(${PROJECT_NAME} 
  ${OpenCV_LIBRARIES} 
  yaml-cpp 
  glog::glog
  rclcpp::rclcpp
  ${std_msgs_TARGETS}
  ${cv_bridge_TARGETS}
)

################################################################################
# Unit tests
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(homography_test test/test_homography.cpp)
  target_link_libraries(homography_test ${PROJECT_NAME})

  ament_add_gtest(math_utils_test test/test_math_utils.cpp)
  target_link_libraries(math_utils_test ${PROJECT_NAME})
endif()

################################################################################
# Other tests
add_executable(vikit_test_timer test/test_timer.cpp)
target_link_libraries(vikit_test_timer ${PROJECT_NAME})

add_executable(vikit_test_common test/test_common.cpp)
target_link_libraries(vikit_test_common ${PROJECT_NAME})

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

ament_export_dependencies(rclcpp std_msgs cv_bridge Eigen3 glog minkindr asr_sdm_vio_cmake yaml-cpp)
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
