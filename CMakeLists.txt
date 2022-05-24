cmake_minimum_required(VERSION 3.22)
project(SchedPlus)
set(CMAKE_CXX_STANDARD 20)
add_executable(SchedPlus main.cpp
        src/components/Port.cpp src/components/Port.h
        src/components/Node.cpp src/components/Node.h
        src/components/NodeImpl.cpp src/components/NodeImpl.h
        src/components/Link.cpp src/components/Link.h
        src/components/Flow.cpp src/components/Flow.h
        src/include/type.h src/Route.cpp src/Route.h
        src/Delivery/DeliveryGuarantee.cpp src/Delivery/DeliveryGuarantee.h
        lib/openGA.hpp
        lib/pugiconfig.hpp
        lib/pugixml.cpp
        lib/pugixml.hpp
        src/solutions/GA.h
        src/solutions/GA_Solution.h
        src/solutions/MyFunctions.h src/solutions/GA_4sw_ring.h src/solutions/GA_line_2_2.h src/solutions/GA_line_2_1.h)

target_link_libraries(SchedPlus
        uuid
        spdlog
        pthread)