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
find_package(glog_vendor REQUIRED)
find_package(glog REQUIRED)
find_package(minkindr REQUIRED)
find_package(svo_cmake REQUIRED)
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

add_library(${PROJECT_NAME} SHARED ${SOURCES} ${HEADERS})

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIR}
)

target_link_libraries(${PROJECT_NAME} yaml-cpp glog::glog)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  std_msgs
  cv_bridge
  glog_vendor
  minkindr
  svo_cmake
  video_kit_common
)

################################################################################
# Tests
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  
  ament_add_gtest(${PROJECT_NAME}_tests
    test/test_cameras.cpp
  )
  target_link_libraries(${PROJECT_NAME}_tests ${PROJECT_NAME})
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
ament_export_dependencies(rclcpp std_msgs cv_bridge Eigen3 glog_vendor minkindr svo_cmake video_kit_common yaml-cpp)
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
