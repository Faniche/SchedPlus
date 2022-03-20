cmake_minimum_required(VERSION 3.20)
project(SchedPlus)
set(CMAKE_CXX_STANDARD 20)
add_executable(SchedPlus main.cpp
        src/components/Port.cpp src/components/Port.h
        src/components/Node.cpp src/components/Node.h
        src/components/NodeImpl.cpp src/components/NodeImpl.h
        src/components/Link.cpp src/components/Link.h
        src/components/Flow.cpp src/components/Flow.h
        src/include/MyVlan.h src/Route.cpp src/Route.h)

target_link_libraries(SchedPlus
        uuid
        spdlog)