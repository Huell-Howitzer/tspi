cmake_minimum_required(VERSION 3.27)
project(tspi)

set(CMAKE_CXX_STANDARD 17)

include_directories(include)

# Include the Eigen library
include_directories(/usr/include/eigen3)

add_executable(tspi main.cpp
        src/StateUpdate.cpp
        include/StateUpdate.h)
