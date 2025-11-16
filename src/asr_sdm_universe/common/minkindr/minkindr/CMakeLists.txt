cmake_minimum_required(VERSION 3.8)
project(minkindr)

# Find ROS 2 build tools
find_package(ament_cmake REQUIRED)

# Find dependencies (from package.xml)
find_package(Eigen3 REQUIRED)
find_package(glog REQUIRED)

# Create an INTERFACE library for the header-only library
add_library(${PROJECT_NAME} INTERFACE)
target_include_directories(${PROJECT_NAME} INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME} INTERFACE
  Eigen3::Eigen
  glog::glog
)

# Install the header files.
install(
  DIRECTORY include/
  DESTINATION include
)

# Install the INTERFACE library
install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
)

# Export dependencies so downstream packages can find them.
ament_export_dependencies(Eigen3 glog)

# Export the include directory.
ament_export_include_directories(include)

# Export the targets
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

# Testing
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(eigen_checks REQUIRED)
  find_package(Threads)

  # Add the test executable using ament_add_gtest
  ament_add_gtest(${PROJECT_NAME}_tests
    test/test-main.cc
    test/test-rotation.cc
    test/test-sim3.cc
    test/test-transformation.cc
    test/test-transformation-2d.cc
  )
  
  # Add include directories for the test
  target_include_directories(${PROJECT_NAME}_tests PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    ${EIGEN3_INCLUDE_DIRS}
  )
  
  # Use ament_target_dependencies to handle eigen_checks
  ament_target_dependencies(${PROJECT_NAME}_tests eigen_checks)
  
  # Link libraries for the test
  target_link_libraries(${PROJECT_NAME}_tests
    glog::glog
    ${CMAKE_THREAD_LIBS_INIT}
  )
endif()

ament_package()
