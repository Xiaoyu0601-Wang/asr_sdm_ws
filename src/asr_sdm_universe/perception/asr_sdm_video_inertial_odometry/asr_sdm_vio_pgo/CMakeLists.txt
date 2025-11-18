cmake_minimum_required(VERSION 3.8)
project(asr_sdm_vio_pgo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(asr_sdm_vio_cmake REQUIRED)
find_package(asr_sdm_vio_common REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED)
find_package(glog REQUIRED)

include(SvoSetup)

set(HEADERS
  include/asr_sdm_vio/pgo/pgo.h
  )

set(SOURCES
  src/pgo.cpp
  )

# Main library
add_library(${PROJECT_NAME} SHARED ${SOURCES} ${HEADERS})
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${CERES_INCLUDE_DIRS}
)
target_link_libraries(${PROJECT_NAME}
  ${CERES_LIBRARIES}
  Eigen3::Eigen
  glog::glog
  rclcpp::rclcpp
  ${std_msgs_TARGETS}
  asr_sdm_vio_common::asr_sdm_vio_common
  ${cv_bridge_TARGETS}
)

# Test executable (disabled)
if(FALSE AND BUILD_TESTING)
  add_executable(test_pgo test/test.cpp)
  target_include_directories(test_pgo PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${CERES_INCLUDE_DIRS}
  )
  target_link_libraries(test_pgo 
    ${PROJECT_NAME}
    ${CERES_LIBRARIES}
    glog::glog
    asr_sdm_vio_common::asr_sdm_vio_common
    rclcpp::rclcpp
    ${std_msgs_TARGETS}
    ${cv_bridge_TARGETS}
  )
endif()

# Install
install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING AND TARGET test_pgo)
  install(TARGETS test_pgo
    RUNTIME DESTINATION lib/${PROJECT_NAME}
  )
endif()

install(DIRECTORY include/
  DESTINATION include
)

# Export
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(rclcpp std_msgs asr_sdm_vio_common cv_bridge Ceres Eigen3 glog)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
