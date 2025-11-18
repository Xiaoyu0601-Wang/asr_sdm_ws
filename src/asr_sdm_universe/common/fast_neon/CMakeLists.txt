cmake_minimum_required(VERSION 3.8)
project(fast)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -Wall -Werror -Wno-unused-variable -Wno-unused-but-set-variable -Wno-unknown-pragmas")

IF(DEFINED ENV{ARM_ARCHITECTURE})
	SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -mfpu=neon -march=armv7-a")
ELSE()
	SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -mmmx -msse -msse -msse2 -msse3 -mssse3 -fomit-frame-pointer")
ENDIF()

set(HEADERS
  include/fast/corner_9.h
  include/fast/corner_10.h
  include/fast/fast.h
  include/fast/faster_corner_utilities.h
  )

set(SOURCES
  src/fast_9.cpp
  src/fast_10.cpp
  src/fast_10_score.cpp
  src/nonmax_3x3.cpp
  )

IF(DEFINED ENV{ARM_ARCHITECTURE})
  LIST(APPEND SOURCES ${PROJECT_SOURCE_DIR}/src/faster_corner_9_neon.cpp)
ELSE()
  LIST(APPEND SOURCES ${PROJECT_SOURCE_DIR}/src/faster_corner_10_sse.cpp)
ENDIF()

# Create shared library
add_library(${PROJECT_NAME} SHARED ${SOURCES} ${HEADERS})

# Add alias for namespaced target
add_library(${PROJECT_NAME}::${PROJECT_NAME} ALIAS ${PROJECT_NAME})

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

################################################################################
install(
    TARGETS ${PROJECT_NAME}
    EXPORT export_${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

install(
    DIRECTORY include/
    DESTINATION include
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
