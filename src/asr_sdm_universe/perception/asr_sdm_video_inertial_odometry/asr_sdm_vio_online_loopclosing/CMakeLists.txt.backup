cmake_minimum_required(VERSION 3.8)
project(asr_sdm_vio_online_loopclosing)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)
find_package(OpenCV REQUIRED)

include(SvoSetup)

set(HEADERS
  include/svo/online_loopclosing/bow.h
  include/svo/online_loopclosing/geometric_verification.h
  include/svo/online_loopclosing/loop_closing.h
  include/svo/online_loopclosing/read_file.h
  include/svo/online_loopclosing/keyframe.h
  include/svo/online_loopclosing/loop_closing_types.h
  include/svo/online_loopclosing/map_alignment.h
  )

set(SOURCES
  src/bow.cpp
  src/geometric_verification.cpp
  src/loop_closing.cpp
  src/read_file.cpp
  src/map_alignment.cpp
  )

#add_executable(svo_online_loopclosing ${SOURCES} ${HEADERS})
cs_add_library(${PROJECT_NAME} ${SOURCES} ${HEADERS})
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBRARIES})

cs_add_library(bow src/bow.cpp include/svo/online_loopclosing/bow.h)
target_link_libraries(bow ${OpenCV_LIBRARIES})

cs_add_executable(createvoc test/createvoc.cpp)
target_link_libraries(createvoc bow)

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
