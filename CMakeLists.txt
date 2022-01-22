cmake_minimum_required(VERSION 3.20)
project(SchedPlus)

set(CMAKE_CXX_STANDARD 20)

add_executable(SchedPlus main.cpp
        src/Components.h
        src/Components.h)
target_link_libraries(SchedPlus
        uuid)