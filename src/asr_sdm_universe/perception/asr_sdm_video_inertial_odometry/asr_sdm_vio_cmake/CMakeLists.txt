cmake_minimum_required(VERSION 3.8)
project(asr_sdm_vio_cmake)

find_package(ament_cmake REQUIRED)

# Configure the extras file for install space
set(CATKIN_PACKAGE_SHARE_DESTINATION share/${PROJECT_NAME})
configure_file(
  cmake/asr_sdm_vio_cmake-extras.cmake.installspace.in
  ${CMAKE_CURRENT_BINARY_DIR}/asr_sdm_vio_cmake-extras.cmake
  @ONLY
)

# Install all included cmake files
install(DIRECTORY cmake/Modules
  DESTINATION share/${PROJECT_NAME}/cmake
)

# Export the cmake directory so other packages can find it
ament_package(
  CONFIG_EXTRAS ${CMAKE_CURRENT_BINARY_DIR}/asr_sdm_vio_cmake-extras.cmake
)
