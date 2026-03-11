cmake_minimum_required(VERSION 3.22)
project(TrajOptArm CXX)

# c++ standard
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Debug)

add_subdirectory(${PROJECT_SOURCE_DIR}/src)
