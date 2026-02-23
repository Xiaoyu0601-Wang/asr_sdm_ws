cmake_minimum_required(VERSION 3.14)
project(gaussian_process_regression VERSION 0.3.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Matplot++ REQUIRED)

# Options
option(BUILD_PYTHON_BINDINGS "Build Python bindings" OFF)
option(LIBGP_BUILD_TESTS "Build libgp tests" OFF)

# Generate version header
configure_file(
    "${PROJECT_SOURCE_DIR}/include/gaussian_process_regression/gp_version.h.in"
    "${PROJECT_BINARY_DIR}/include/gaussian_process_regression/gp_version.h"
    @ONLY
)

# ---- Core GP library (static) ----
add_library(gp STATIC
    src/gpr_algorithm/gp.cc
    src/gpr_algorithm/gp_utils.cc
    src/gpr_algorithm/sampleset.cc
    src/gpr_algorithm/rprop.cc
    src/gpr_algorithm/cg.cc
    src/gpr_algorithm/input_dim_filter.cc
    src/gpr_algorithm/cov.cc
    src/gpr_algorithm/cov_factory.cc
    src/gpr_algorithm/cov_linear_ard.cc
    src/gpr_algorithm/cov_linear_one.cc
    src/gpr_algorithm/cov_matern3_iso.cc
    src/gpr_algorithm/cov_matern5_iso.cc
    src/gpr_algorithm/cov_noise.cc
    src/gpr_algorithm/cov_periodic.cc
    src/gpr_algorithm/cov_periodic_matern3_iso.cc
    src/gpr_algorithm/cov_prod.cc
    src/gpr_algorithm/cov_rq_iso.cc
    src/gpr_algorithm/cov_se_ard.cc
    src/gpr_algorithm/cov_se_iso.cc
    src/gpr_algorithm/cov_sum.cc
)

target_include_directories(gp
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

target_link_libraries(gp PUBLIC Eigen3::Eigen)

# ---- ROS2 executable node ----
add_executable(gp_example_dense src/gp_example_dense.cpp)
target_link_libraries(gp_example_dense gp Matplot++::matplot)
ament_target_dependencies(gp_example_dense rclcpp)

# ---- Python bindings (optional) ----
if(BUILD_PYTHON_BINDINGS)
    set(PYBIND11_FINDPYTHON ON)
    find_package(pybind11 CONFIG REQUIRED)
    find_package(Python COMPONENTS Interpreter Development REQUIRED)

    pybind11_add_module(libgp_cpp src/gpr_algorithm/bindings.cpp)
    target_link_libraries(libgp_cpp PRIVATE gp Eigen3::Eigen)
    set_target_properties(libgp_cpp PROPERTIES
        OUTPUT_NAME "libgp_cpp"
        LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/libgp"
    )

    install(TARGETS libgp_cpp
        LIBRARY DESTINATION "libgp"
        ARCHIVE DESTINATION "libgp"
    )
    install(FILES "${CMAKE_CURRENT_SOURCE_DIR}/libgp/__init__.py"
            DESTINATION "libgp")
endif()

# ---- Tests (optional) ----
if(LIBGP_BUILD_TESTS)
    enable_testing()
    find_package(GTest REQUIRED)

    function(add_gp_test name)
        add_executable(${name} tests/${name}.cc)
        target_link_libraries(${name} PRIVATE gp GTest::GTest GTest::Main)
        add_test(NAME ${name} COMMAND ${name})
    endfunction()

    add_gp_test(test_gp_regression)
    add_gp_test(test_log_likelihood)
    add_gp_test(test_cov_factory)
    add_gp_test(test_covariance_functions)
    add_gp_test(test_gp_utils)
    add_gp_test(test_optimizer)
endif()

# ---- Install ----
install(TARGETS gp
    EXPORT libgp-targets
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION lib/${PROJECT_NAME}
    INCLUDES DESTINATION include
)

install(TARGETS gp_example_dense
    DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
    DESTINATION include
    FILES_MATCHING PATTERN "*.h"
    PATTERN "*.h.in" EXCLUDE
)

install(FILES "${CMAKE_CURRENT_BINARY_DIR}/include/gaussian_process_regression/gp_version.h"
    DESTINATION include/gaussian_process_regression
)

# ---- Export ----
install(EXPORT libgp-targets
    FILE libgpTargets.cmake
    NAMESPACE libgp::
    DESTINATION lib/cmake/libgp
)

include(CMakePackageConfigHelpers)
write_basic_package_version_file(
    "${CMAKE_CURRENT_BINARY_DIR}/libgpConfigVersion.cmake"
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY SameMajorVersion
)

configure_package_config_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/contrib/libgpConfig.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}/libgpConfig.cmake"
    INSTALL_DESTINATION lib/cmake/libgp
)

install(FILES
    "${CMAKE_CURRENT_BINARY_DIR}/libgpConfig.cmake"
    "${CMAKE_CURRENT_BINARY_DIR}/libgpConfigVersion.cmake"
    DESTINATION lib/cmake/libgp
)

ament_export_include_directories(include)
ament_export_libraries(gp)
ament_package()
