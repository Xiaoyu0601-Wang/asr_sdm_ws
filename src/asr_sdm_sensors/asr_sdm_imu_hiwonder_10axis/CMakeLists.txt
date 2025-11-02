cmake_minimum_required(VERSION 3.20)
project(asr_sdm_imu_hiwonder_10axis)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(data_structure REQUIRED)
find_package(periphery REQUIRED)

include_directories(
  include
  /usr/local/include
  ${data_structure_INCLUDE_DIRS}
)

# library for IMU driver implementation
add_library(${PROJECT_NAME} SHARED
  src/imu_hiwonder_10axis.cpp
)
target_link_libraries(${PROJECT_NAME} PUBLIC
  rclcpp::rclcpp
  data_structure::data_structure
  periphery
)

install(TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME} # Recommended to use a different export name
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin # For shared libraries
)

# install headers
install(DIRECTORY include/ DESTINATION include)

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

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
# ament_export_dependencies(ament_cmake)
ament_export_include_directories(include)
ament_package()
