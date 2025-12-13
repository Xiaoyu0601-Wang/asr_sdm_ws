cmake_minimum_required(VERSION 3.8)
project(fast)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Based on the ARM_ARCHITECTURE environment variable
# Set IS_ARM to build on the Odroid board
if(DEFINED ENV{ARM_ARCHITECTURE})
  message(STATUS "ARM_ARCHITECTURE environment variable set to $ENV{ARM_ARCHITECTURE}")
  set(IS_ARM TRUE)
else()
  set(IS_ARM FALSE)
endif()

option(BUILD_TESTING "Build tests" ON)

# Find dependencies
find_package(ament_cmake REQUIRED)

# Build type and flags
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

if(IS_ARM)
  add_compile_options(-mfpu=neon -march=armv7-a)
else()
  add_compile_options(-mmmx -msse -msse2 -msse3 -mssse3 -fomit-frame-pointer)
endif()

# Add Sources
set(SOURCES
  src/fast_10.cpp
  src/fast_10_score.cpp
  src/nonmax_3x3.cpp
)

if(IS_ARM)
  # list(APPEND SOURCES src/faster_corner_9_neon.cpp)
else()
  list(APPEND SOURCES src/faster_corner_10_sse.cpp)
endif()

# Add library
add_library(${PROJECT_NAME} SHARED ${SOURCES})

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Add Tests
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(OpenCV QUIET)
  if(OpenCV_FOUND)
    message(STATUS "Building Fast Test")
    add_executable(fast_test test/test.cpp)
    target_compile_definitions(fast_test PRIVATE TEST_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test/data")
    target_link_libraries(fast_test ${PROJECT_NAME} ${OpenCV_LIBS})
    target_include_directories(fast_test PRIVATE
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    )

    install(TARGETS fast_test
      DESTINATION lib/${PROJECT_NAME}
    )
  else()
    message(STATUS "OpenCV not found, skipping fast_test")
  endif()
endif()

# Install headers
install(
  DIRECTORY include/
  DESTINATION include
)

# Install library
install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# Export targets
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_package()
