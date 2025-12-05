cmake_minimum_required(VERSION 3.8)
project(minkindr)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(glog REQUIRED)

# Include headers
# include_directories(
#   include
# )

# Since this is a header-only library, we don't use add_library().
# We just need to install the headers and export the dependencies.
# Declare a header-only "library"
add_library(${PROJECT_NAME} INTERFACE)
target_include_directories(${PROJECT_NAME} INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# export the interface target so other packages can link against `data_structure`
install(TARGETS ${PROJECT_NAME}
  EXPORT "export_${PROJECT_NAME}"
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib
  INCLUDES DESTINATION include
)

# Install the header files.
install(
  DIRECTORY include/
  DESTINATION include
)

# Export dependencies so downstream packages can find them.
# ament_export_dependencies(Eigen3 glog)

# Export the include directory.
ament_export_include_directories(include)

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
  # ament_target_dependencies(${PROJECT_NAME}_tests eigen_checks)
  
  # Link libraries for the test
  target_link_libraries(${PROJECT_NAME}_tests
    glog::glog
    eigen_checks::eigen_checks
    ${CMAKE_THREAD_LIBS_INIT}
  )
endif()

ament_export_targets("export_${PROJECT_NAME}")
ament_package()
