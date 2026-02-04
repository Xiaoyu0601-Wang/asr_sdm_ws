cmake_minimum_required(VERSION 3.8)
project(vikit_cameras)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(glog REQUIRED)
find_package(minkindr REQUIRED)
find_package(asr_sdm_cmake REQUIRED)
find_package(vikit_common REQUIRED)
find_package(yaml-cpp REQUIRED)

include(SvoSetup)

set(HEADERS
  include/vikit/cameras.h
  include/vikit/cameras/camera_geometry_base.h
  include/vikit/cameras/implementation/camera_geometry_base.hpp
  include/vikit/cameras/no_distortion.h
  include/vikit/cameras/atan_distortion.h
  include/vikit/cameras/equidistant_distortion.h
  include/vikit/cameras/equidistant_fisheye_geometry.h
  include/vikit/cameras/equidistant_fisheye_projection.h
  include/vikit/cameras/omni_geometry.h
  include/vikit/cameras/omni_projection.h
  include/vikit/cameras/radial_tangential_distortion.h
  include/vikit/cameras/pinhole_projection.h
  include/vikit/cameras/implementation/pinhole_projection.hpp
  include/vikit/cameras/camera_geometry.h
  include/vikit/cameras/implementation/camera_geometry.hpp
  include/vikit/cameras/camera_factory.h
  include/vikit/cameras/ncamera.h
  include/vikit/cameras/yaml/camera-yaml-serialization.h
  include/vikit/cameras/yaml/ncamera-yaml-serialization.h
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
  ${minkindr_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME} 
  yaml-cpp 
  glog::glog
  rclcpp::rclcpp
  ${std_msgs_TARGETS}
  ${cv_bridge_TARGETS}
  vikit_common::vikit_common
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
  )
  ament_target_dependencies(${PROJECT_NAME}_tests
    eigen_checks
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
ament_export_dependencies(rclcpp std_msgs cv_bridge Eigen3 glog minkindr asr_sdm_cmake vikit_common yaml-cpp)
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
