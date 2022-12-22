cmake_minimum_required(VERSION 3.24)
project(3dof_thing)

set(CMAKE_CXX_STANDARD 11)

include_directories(include)

add_executable(3dof_thing src/main.cpp src/Program.cpp src/Robot.cpp)
