# Copyright (c) 2015, Autonomous Systems Lab, ETH Zurich
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the <organization> nor the
#       names of its contributors may be used to endorse or promote products
#       derived from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
cmake_minimum_required(VERSION 3.8)
project(eigen_checks)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(glog REQUIRED)

# Create library
add_library(${PROJECT_NAME} SHARED src/eigen-checks.cc)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
  ${EIGEN3_INCLUDE_DIR}
)
target_link_libraries(${PROJECT_NAME} PUBLIC glog::glog Eigen3::Eigen)

# Install library
install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install headers
install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

# Export dependencies
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3 glog)
ament_export_include_directories(include/${PROJECT_NAME})

# Tests
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(Threads REQUIRED)

  ament_add_gtest(test_gtest_near test/test_gtest-near.cc)
  target_link_libraries(test_gtest_near ${PROJECT_NAME} ${CMAKE_THREAD_LIBS_INIT})

  ament_add_gtest(test_gtest_equal test/test_gtest-equal.cc)
  target_link_libraries(test_gtest_equal ${PROJECT_NAME} ${CMAKE_THREAD_LIBS_INIT})

  ament_add_gtest(test_gtest_zero test/test_gtest-zero.cc)
  target_link_libraries(test_gtest_zero ${PROJECT_NAME} ${CMAKE_THREAD_LIBS_INIT})

  ament_add_gtest(test_gtest_equal_double test/test_gtest-equal-double.cc)
  target_link_libraries(test_gtest_equal_double ${PROJECT_NAME} ${CMAKE_THREAD_LIBS_INIT})

  ament_add_gtest(test_glog_near test/test_glog-near.cc)
  target_link_libraries(test_glog_near ${PROJECT_NAME} ${CMAKE_THREAD_LIBS_INIT})

  ament_add_gtest(test_glog_equal test/test_glog-equal.cc)
  target_link_libraries(test_glog_equal ${PROJECT_NAME} ${CMAKE_THREAD_LIBS_INIT})

  ament_add_gtest(test_glog_zero test/test_glog-zero.cc)
  target_link_libraries(test_glog_zero ${PROJECT_NAME} ${CMAKE_THREAD_LIBS_INIT})

  ament_add_gtest(test_glog_equal_double test/test_glog-equal-double.cc)
  target_link_libraries(test_glog_equal_double ${PROJECT_NAME} ${CMAKE_THREAD_LIBS_INIT})

  ament_add_gtest(test_glog_zero_sized_types test/test_glog-zero-sized-types.cc)
  target_link_libraries(test_glog_zero_sized_types ${PROJECT_NAME} ${CMAKE_THREAD_LIBS_INIT})

  ament_add_gtest(test_gtest_zero_sized_types test/test_gtest-zero-sized-types.cc)
  target_link_libraries(test_gtest_zero_sized_types ${PROJECT_NAME} ${CMAKE_THREAD_LIBS_INIT})
endif()

ament_package()
