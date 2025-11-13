cmake_minimum_required(VERSION 3.8)
project(asr_sdm_vio_test_utils)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

include(SvoSetup)

################################################################################
set(HEADERS
  include/svo/test_utils/test_utils.h
  include/svo/test_utils/imu_simulation.h
  include/svo/test_utils/synthetic_dataset.h
  include/svo/test_utils/simple_raytracer.h
  include/svo/test_utils/trajectory_generator.h
  include/svo/test_utils/point_cloud_generator.h
)

set(SOURCES
  src/test_utils.cpp
  src/imu_simulation.cpp
  src/synthetic_dataset.cpp
  src/simple_raytracer.cpp
  src/trajectory_generator.cpp
  src/point_cloud_generator.cpp
)

cs_add_library(${PROJECT_NAME} ${SOURCES} ${HEADERS})

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
