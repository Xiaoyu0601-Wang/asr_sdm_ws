cmake_minimum_required(VERSION 3.8)
project(dbow2_vendor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(OpenCV REQUIRED)

# Check if DBoW2 is already installed
find_library(DBOW2_LIBRARY NAMES DBoW2)
find_path(DBOW2_INCLUDE_DIR NAMES DBoW2/DBoW2.h)

if(DBOW2_LIBRARY AND DBOW2_INCLUDE_DIR)
  message(STATUS "Found DBoW2, skipping build from source")
  message(STATUS "  Library: ${DBOW2_LIBRARY}")
  message(STATUS "  Include: ${DBOW2_INCLUDE_DIR}")
  
  # Create an interface library to export
  add_library(${PROJECT_NAME} INTERFACE)
  target_include_directories(${PROJECT_NAME} INTERFACE ${DBOW2_INCLUDE_DIR})
  target_link_libraries(${PROJECT_NAME} INTERFACE ${DBOW2_LIBRARY})
  
  install(TARGETS ${PROJECT_NAME}
    EXPORT ${PROJECT_NAME}Targets
  )
  
else()
  message(STATUS "DBoW2 not found, building from source")
  
  include(ExternalProject)

  set(DBOW2_INSTALL_DIR ${CMAKE_CURRENT_BINARY_DIR}/dbow2_install)

  # Create include directory
  file(MAKE_DIRECTORY ${DBOW2_INSTALL_DIR}/include)

  ExternalProject_Add(dbow2_src
    GIT_REPOSITORY https://github.com/dorian3d/DBoW2.git
    GIT_TAG master
    UPDATE_COMMAND ""
    CMAKE_ARGS
      -DCMAKE_INSTALL_PREFIX=${DBOW2_INSTALL_DIR}
      -DCMAKE_BUILD_TYPE=Release
      -DCMAKE_CXX_FLAGS=-fPIC
      -DCMAKE_POSITION_INDEPENDENT_CODE=ON
      -DOpenCV_DIR=${OpenCV_DIR}
    BUILD_COMMAND ${CMAKE_COMMAND} --build . -- -j8
    INSTALL_COMMAND ${CMAKE_COMMAND} --build . --target install
  )

  # Create a dummy source file for the library
  file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/dependency_tracker.cc
    "// Dummy file for dependency tracking\n")

  # Create wrapper library
  add_library(${PROJECT_NAME} SHARED ${CMAKE_CURRENT_BINARY_DIR}/dependency_tracker.cc)
  add_dependencies(${PROJECT_NAME} dbow2_src)
  
  target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${DBOW2_INSTALL_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  
  target_link_libraries(${PROJECT_NAME}
    ${DBOW2_INSTALL_DIR}/lib/libDBoW2${CMAKE_SHARED_LIBRARY_SUFFIX}
    ${OpenCV_LIBS}
  )

  # Install headers
  install(DIRECTORY ${DBOW2_INSTALL_DIR}/include/
          DESTINATION include
          FILES_MATCHING PATTERN "*.h")

  # Install DBoW2 library
  install(FILES ${DBOW2_INSTALL_DIR}/lib/libDBoW2${CMAKE_SHARED_LIBRARY_SUFFIX}
          DESTINATION lib)

  # Install wrapper library
  install(TARGETS ${PROJECT_NAME}
    EXPORT ${PROJECT_NAME}Targets
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
  )

endif()

# Export information
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME} DBoW2)
ament_export_dependencies(OpenCV)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
