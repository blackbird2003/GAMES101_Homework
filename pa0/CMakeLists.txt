cmake_minimum_required (VERSION 2.8.11)
project (Transformation)

find_package(Eigen3 REQUIRED)
include_directories(EIGEN3_INCLUDE_DIR)

set(CMAKE_CXX_STANDARD 17)

add_executable (Transformation main.cpp)
