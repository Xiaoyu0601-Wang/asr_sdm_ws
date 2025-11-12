cmake_minimum_required(VERSION 3.8)
project(video_kit_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(video_kit_common REQUIRED)




set(HEADERS
  include/video_kit/img_type_conversion.h
  include/video_kit/output_helper.h
  include/video_kit/params_helper.h
  )

set(SOURCES
  src/output_helper.cpp
  src/img_type_conversion.cpp
  )

# Create vikit library
add_library(${PROJECT_NAME} SHARED ${SOURCES})
ament_target_dependencies(${PROJECT_NAME} rclcpp std_msgs video_kit_common)
target_link_libraries(${PROJECT_NAME} Eigen3::Eigen)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

################################################################################
install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)
install(
  DIRECTORY include/
  DESTINATION include
)

ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(rclcpp std_msgs video_kit_common)


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
