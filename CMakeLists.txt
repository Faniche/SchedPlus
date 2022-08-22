cmake_minimum_required(VERSION 3.22)
project(SchedPlus)
set(CMAKE_CXX_STANDARD 20)

add_subdirectory(lib/CLI11
                lib/spdlog)
add_executable(SchedPlus_nowait main.cpp
        src/components/Port.cpp src/components/Port.h
        src/components/Node.cpp src/components/Node.h
        src/components/NodeImpl.cpp src/components/NodeImpl.h
        src/components/Link.cpp src/components/Link.h
        src/components/Flow.cpp src/components/Flow.h
        src/include/type.h src/Route.cpp src/Route.h
        src/Delivery/DeliveryGuarantee.cpp src/Delivery/DeliveryGuarantee.h
        lib/openGA/openGA.hpp
        lib/pugixml/pugiconfig.hpp
        lib/pugixml/pugixml.cpp
        lib/pugixml/pugixml.hpp
        lib/json/json.hpp
        src/solutions/GA.h
        src/solutions/GA_Solution.h
        src/solutions/GA_4sw_ring.h
        src/solutions/GA_line_2_2.h
        src/solutions/GA_line_2_1.h
        src/event/Event.h
        src/solutions/NoWait.h src/solutions/GA_tree.h)

add_executable(SchedPlus_wait main.cpp
        src/components/Port.cpp src/components/Port.h
        src/components/Node.cpp src/components/Node.h
        src/components/NodeImpl.cpp src/components/NodeImpl.h
        src/components/Link.cpp src/components/Link.h
        src/components/Flow.cpp src/components/Flow.h
        src/include/type.h src/Route.cpp src/Route.h
        src/Delivery/DeliveryGuarantee.cpp src/Delivery/DeliveryGuarantee.h
        lib/openGA/openGA.hpp
        lib/pugixml/pugiconfig.hpp
        lib/pugixml/pugixml.cpp
        lib/pugixml/pugixml.hpp
        lib/json/json.hpp
        src/solutions/GA.h
        src/solutions/GA_Solution.h
        src/solutions/GA_4sw_ring.h
        src/solutions/GA_line_2_2.h
        src/solutions/GA_line_2_1.h
        src/event/Event.h
        src/solutions/Wait.h src/solutions/GA_tree.h)



target_link_libraries(SchedPlus_nowait PUBLIC spdlog
        uuid
        pthread
        CLI11::CLI11)

target_link_libraries(SchedPlus_wait PUBLIC spdlog
        uuid
        pthread
        CLI11::CLI11)
#
target_include_directories(SchedPlus_wait PUBLIC
        "${PROJECT_BINARY_DIR}"
        "${PROJECT_SOURCE_DIR}/lib/spdlog")
