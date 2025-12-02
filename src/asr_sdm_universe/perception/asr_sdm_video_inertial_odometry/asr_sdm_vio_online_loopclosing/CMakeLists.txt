cmake_minimum_required(VERSION 3.8)
project(asr_sdm_vio_online_loopclosing)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
  # Disable specific warnings that cause issues with DBoW2 and unused parameters
  add_compile_options(-Wno-overloaded-virtual -Wno-unused-parameter)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)

find_package(opengv REQUIRED)
find_package(asr_sdm_vio_cmake REQUIRED)
find_package(asr_sdm_vio_common REQUIRED)
find_package(asr_sdm_rpg_common REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(video_kit_cameras REQUIRED)
find_package(asr_sdm_vio_pgo REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(glog REQUIRED)
find_package(CURL REQUIRED)

# Try to find DBoW2 from dbow2_vendor first
find_package(dbow2_vendor QUIET)
if(dbow2_vendor_FOUND)
  message(STATUS "Found dbow2_vendor package")
  set(HAVE_DBOW2 TRUE)
  set(DBOW2_INCLUDE_DIR "${dbow2_vendor_DIR}/../../../include")
  set(DBOW2_LIBRARY "${dbow2_vendor_DIR}/../../../lib/libDBoW2.so")
else()
  # Fallback to manual search
  find_path(DBOW2_INCLUDE_DIR DBoW2/DBoW2.h
    PATHS
      /usr/include
      /usr/local/include
      ${CMAKE_INSTALL_PREFIX}/include
  )

  find_library(DBOW2_LIBRARY
    NAMES DBoW2 libDBoW2
    PATHS
      /usr/lib
      /usr/local/lib
      ${CMAKE_INSTALL_PREFIX}/lib
  )
  
  if(NOT DBOW2_INCLUDE_DIR OR NOT DBOW2_LIBRARY)
    set(HAVE_DBOW2 FALSE)
  else()
    set(HAVE_DBOW2 TRUE)
  endif()
endif()

# Try to find opengv
find_path(OPENGV_INCLUDE_DIR opengv/types.hpp
  PATHS
    /usr/include
    /usr/local/include
    ${CMAKE_INSTALL_PREFIX}/include
)

find_library(OPENGV_LIBRARY
  NAMES opengv libopengv
  PATHS
    /usr/lib
    /usr/local/lib
    ${CMAKE_INSTALL_PREFIX}/lib
    /usr/lib/x86_64-linux-gnu
)

if(HAVE_DBOW2)
  message(STATUS "Found DBoW2: ${DBOW2_LIBRARY}")
else()
  message(WARNING "DBoW2 not found. Some features will be disabled.")
endif()

if(NOT OPENGV_INCLUDE_DIR OR NOT OPENGV_LIBRARY)
  message(WARNING "opengv not found. Some features will be disabled.")
  set(HAVE_OPENGV FALSE)
else()
  message(STATUS "Found opengv: ${OPENGV_LIBRARY}")
  set(HAVE_OPENGV TRUE)
endif()

include(SvoSetup)

set(HEADERS
  include/asr_sdm_vio/online_loopclosing/bow.h
  include/asr_sdm_vio/online_loopclosing/geometric_verification.h
  include/asr_sdm_vio/online_loopclosing/loop_closing.h
  include/asr_sdm_vio/online_loopclosing/read_file.h
  include/asr_sdm_vio/online_loopclosing/keyframe.h
  include/asr_sdm_vio/online_loopclosing/loop_closing_types.h
  include/asr_sdm_vio/online_loopclosing/map_alignment.h
  )

set(SOURCES
  src/bow.cpp
  src/geometric_verification.cpp
  src/loop_closing.cpp
  src/read_file.cpp
  src/map_alignment.cpp
  )

# Main library
add_library(${PROJECT_NAME} SHARED ${SOURCES} ${HEADERS})
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME} 
  PUBLIC
    ${OpenCV_LIBRARIES}
    Eigen3::Eigen
    glog::glog
    CURL::libcurl
    rclcpp::rclcpp
    ${std_msgs_TARGETS}
    ament_index_cpp::ament_index_cpp
    asr_sdm_vio_common::asr_sdm_vio_common
    asr_sdm_rpg_common::asr_sdm_rpg_common
    ${cv_bridge_TARGETS}
    video_kit_cameras::video_kit_cameras
    asr_sdm_vio_pgo::asr_sdm_vio_pgo
)

if(HAVE_DBOW2)
  target_include_directories(${PROJECT_NAME} PUBLIC ${DBOW2_INCLUDE_DIR})
  target_link_libraries(${PROJECT_NAME} PUBLIC ${DBOW2_LIBRARY})
endif()


target_include_directories(${PROJECT_NAME} PUBLIC ${OPENGV_INCLUDE_DIR})
target_link_libraries(${PROJECT_NAME} PUBLIC ${OPENGV_LIBRARY})


# Bow library
add_library(bow SHARED src/bow.cpp include/asr_sdm_vio/online_loopclosing/bow.h)
target_include_directories(bow PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(bow 
  PUBLIC
    ${OpenCV_LIBRARIES}
    Eigen3::Eigen
    glog::glog
    CURL::libcurl
    rclcpp::rclcpp
    ${std_msgs_TARGETS}
    ament_index_cpp::ament_index_cpp
    asr_sdm_vio_common::asr_sdm_vio_common
    asr_sdm_rpg_common::asr_sdm_rpg_common
    ${cv_bridge_TARGETS}
    video_kit_cameras::video_kit_cameras
    asr_sdm_vio_pgo::asr_sdm_vio_pgo
)

if(HAVE_DBOW2)
  target_include_directories(bow PUBLIC ${DBOW2_INCLUDE_DIR})
  target_link_libraries(bow PUBLIC ${DBOW2_LIBRARY})
endif()

if(HAVE_OPENGV)
  target_include_directories(bow PUBLIC ${OPENGV_INCLUDE_DIR})
  target_link_libraries(bow PUBLIC ${OPENGV_LIBRARY})
endif()

# Createvoc executable (disabled due to glog linking issues)
if(FALSE)
add_executable(createvoc test/createvoc.cpp)
target_include_directories(createvoc PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(createvoc PRIVATE bow glog::glog ${OpenCV_LIBRARIES} Eigen3::Eigen CURL::libcurl rclcpp::rclcpp ${std_msgs_TARGETS} ament_index_cpp::ament_index_cpp asr_sdm_vio_common::asr_sdm_vio_common asr_sdm_rpg_common::asr_sdm_rpg_common ${cv_bridge_TARGETS} video_kit_cameras::video_kit_cameras asr_sdm_vio_pgo::asr_sdm_vio_pgo)
endif()

# Install
install(TARGETS ${PROJECT_NAME} bow
  EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(TARGET createvoc)
install(TARGETS createvoc
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
endif()

install(DIRECTORY include/
  DESTINATION include
)

# Export
ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME} bow)
ament_export_dependencies(rclcpp std_msgs ament_index_cpp asr_sdm_vio_common asr_sdm_rpg_common cv_bridge video_kit_cameras asr_sdm_vio_pgo CURL)

ament_package()
