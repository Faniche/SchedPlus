cmake_minimum_required(VERSION 3.20)
project(SchedPlus)

set(CMAKE_CXX_STANDARD 20)

add_executable(SchedPlus main.cpp
        src/lib/components/Port.cpp src/lib/components/Port.h
        src/lib/components/Node.cpp src/lib/components/Node.h
        src/lib/components/NodeImpl.cpp src/lib/components/NodeImpl.h
        src/lib/components/Link.cpp src/lib/components/Link.h
        src/lib/components/Flow.cpp src/lib/components/Flow.h
        src/lib/Route.h
        src/lib/components/MyVlan.h)
target_link_libraries(SchedPlus
        uuid
        spdlog)