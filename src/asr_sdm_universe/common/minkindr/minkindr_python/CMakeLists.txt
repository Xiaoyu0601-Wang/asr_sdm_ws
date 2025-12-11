cmake_minimum_required(VERSION 3.8)
project(minkindr_python)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Install the minkindr Python package
ament_python_install_package(minkindr)

ament_package()
