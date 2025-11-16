cmake_minimum_required(VERSION 3.8)
project(video_kit_cameras)

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
find_package(video_kit_common REQUIRED)
find_package(yaml-cpp REQUIRED)

include(SvoSetup)

set(HEADERS
  include/video_kit/cameras.h
  include/video_kit/cameras/camera_geometry_base.h
  include/video_kit/cameras/implementation/camera_geometry_base.hpp
  include/video_kit/cameras/no_distortion.h
  include/video_kit/cameras/atan_distortion.h
  include/video_kit/cameras/equidistant_distortion.h
  include/video_kit/cameras/equidistant_fisheye_geometry.h
  include/video_kit/cameras/equidistant_fisheye_projection.h
  include/video_kit/cameras/omni_geometry.h
  include/video_kit/cameras/omni_projection.h
  include/video_kit/cameras/radial_tangential_distortion.h
  include/video_kit/cameras/pinhole_projection.h
  include/video_kit/cameras/implementation/pinhole_projection.hpp
  include/video_kit/cameras/camera_geometry.h
  include/video_kit/cameras/implementation/camera_geometry.hpp
  include/video_kit/cameras/camera_factory.h
  include/video_kit/cameras/ncamera.h
  include/video_kit/cameras/yaml/camera-yaml-serialization.h
  include/video_kit/cameras/yaml/ncamera-yaml-serialization.h
  )

set(SOURCES
  src/camera_factory.cpp
  src/ncamera.cpp
  src/camera_geometry_base.cpp
  src/camera_yaml_serialization.cpp
  src/equidistant_fisheye_geometry.cpp
  src/equidistant_fisheye_projection.cpp
  src/omni_geometry.cpp
  src/omni_projection.cpp
  src/ncamera_yaml_serialization.cpp
  )

# Create a symbolic link for vikit includes at configure time
# This allows #include <vikit/...> to work with files in include/video_kit/
if(NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/include/vikit")
  execute_process(
    COMMAND ${CMAKE_COMMAND} -E create_symlink
      ${CMAKE_CURRENT_SOURCE_DIR}/include/video_kit
      ${CMAKE_CURRENT_SOURCE_DIR}/include/vikit
    RESULT_VARIABLE SYMLINK_RESULT
  )
  if(NOT SYMLINK_RESULT EQUAL 0)
    message(WARNING "Failed to create symbolic link for vikit includes")
  endif()
endif()

add_library(${PROJECT_NAME} SHARED ${SOURCES} ${HEADERS})

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIR}
  ${minkindr_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME} 
  yaml-cpp 
  glog::glog
  rclcpp::rclcpp
  ${std_msgs_TARGETS}
  ${cv_bridge_TARGETS}
  video_kit_common::video_kit_common
)

################################################################################
# Tests
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(eigen_checks REQUIRED)
  
  ament_add_gtest(${PROJECT_NAME}_tests
    test/test_cameras.cpp
  )
  find_package(CURL REQUIRED)
  target_link_libraries(${PROJECT_NAME}_tests 
    ${PROJECT_NAME} 
    ${CURL_LIBRARIES}
    eigen_checks::eigen_checks
  )
endif()

################################################################################
# Install
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

################################################################################
# Export
ament_export_dependencies(rclcpp std_msgs cv_bridge Eigen3 glog minkindr asr_sdm_vio_cmake video_kit_common yaml-cpp)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
