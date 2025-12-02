cmake_minimum_required(VERSION 3.8)
project(ceres_vendor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)

# Check if Ceres is already installed
find_package(Ceres QUIET)

if(Ceres_FOUND)
  message(STATUS "Found Ceres ${Ceres_VERSION}, skipping build from source")
  
  # Just export the found Ceres
  ament_export_dependencies(Ceres)
  
else()
  message(STATUS "Ceres not found, building from source")
  
  include(ExternalProject)

  set(VERSION 1.14.0)
  set(CERES_INSTALL_DIR ${CMAKE_INSTALL_PREFIX})

  # Create include directory
  file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/ceres_install/include)

  ExternalProject_Add(ceres_src
    GIT_REPOSITORY https://github.com/ceres-solver/ceres-solver.git
    GIT_TAG ${VERSION}
    UPDATE_COMMAND ""
    CMAKE_ARGS
      -DCMAKE_INSTALL_PREFIX=${CMAKE_CURRENT_BINARY_DIR}/ceres_install
      -DBUILD_DOCUMENTATION=OFF
      -DBUILD_EXAMPLES=OFF
      -DBUILD_TESTING=OFF
      -DBUILD_SHARED_LIBS=ON
      -DCMAKE_BUILD_TYPE=Release
      -DCMAKE_CXX_FLAGS=-fPIC
      -DCMAKE_POSITION_INDEPENDENT_CODE=ON
    BUILD_COMMAND ${CMAKE_COMMAND} --build . -- -j8
    INSTALL_COMMAND ${CMAKE_COMMAND} --build . --target install
  )

  # Install headers
  install(DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/ceres_install/include/
          DESTINATION include
          FILES_MATCHING PATTERN "*.h")

  # Install libraries
  install(DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/ceres_install/lib/
          DESTINATION lib
          FILES_MATCHING 
          PATTERN "libceres*"
          PATTERN "cmake" EXCLUDE)

  # Export information
  ament_export_include_directories(include)
  ament_export_libraries(ceres)
  
endif()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
