cmake_minimum_required(VERSION 3.8)
project(asr_sdm_vio_imu)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(glog REQUIRED)
find_package(gflags REQUIRED)
find_package(minkindr REQUIRED)
find_package(asr_sdm_vio_common REQUIRED)

#############
# LIBRARIES #
#############
set(HEADERS
  include/asr_sdm_vio_imu/vio_common/logging.hpp
  include/asr_sdm_vio_imu/vio_common/matrix_operations.hpp
  include/asr_sdm_vio_imu/vio_common/matrix.hpp
  include/asr_sdm_vio_imu/vio_common/test_utils.hpp
  include/asr_sdm_vio_imu/vio_common/backend_types.hpp
  )

set(SOURCES
  src/vio_common.cpp
  src/test_utils.cpp
  )

# Create a symbolic link for svo includes at configure time
# This allows #include <svo/...> to work with files in include/asr_sdm_vio_imu/
if(NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/include/svo")
  execute_process(
    COMMAND ${CMAKE_COMMAND} -E create_symlink
      ${CMAKE_CURRENT_SOURCE_DIR}/include/asr_sdm_vio_imu
      ${CMAKE_CURRENT_SOURCE_DIR}/include/svo
    RESULT_VARIABLE SYMLINK_RESULT
  )
  if(NOT SYMLINK_RESULT EQUAL 0)
    message(WARNING "Failed to create symbolic link for svo includes")
  endif()
endif()

# Create a symbolic link for svo includes at configure time
# This allows #include <svo/...> to work with files in include/asr_sdm_vio_imu/
if(NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/include/svo")
  execute_process(
    COMMAND ${CMAKE_COMMAND} -E create_symlink
      ${CMAKE_CURRENT_SOURCE_DIR}/include/asr_sdm_vio_imu
      ${CMAKE_CURRENT_SOURCE_DIR}/include/svo
    RESULT_VARIABLE SYMLINK_RESULT
  )
  if(NOT SYMLINK_RESULT EQUAL 0)
    message(WARNING "Failed to create symbolic link for svo includes")
  endif()
endif()

add_library(${PROJECT_NAME} ${SOURCES} ${HEADERS})

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
  ${minkindr_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME}
  glog::glog
  gflags
  rclcpp::rclcpp
  ${std_msgs_TARGETS}
  asr_sdm_vio_common::asr_sdm_vio_common
)

##########
# INSTALL #
##########
install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

##########
# EXPORT #
##########
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp
  std_msgs
  Eigen3
  minkindr
  asr_sdm_vio_common
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
