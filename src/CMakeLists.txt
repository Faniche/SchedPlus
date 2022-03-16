set(BINARY ${CMAKE_PROJECT_NAME})

file(GLOB_RECURSE SOURCES LIST_DIRECTORIES true *.h *.cpp)
set(SOURCES ${SOURCES})
#add_executable(SchedPlus main.cpp
#        components/Port.cpp components/Port.h
#        components/Node.cpp components/Node.h
#        components/NodeImpl.cpp components/NodeImpl.h
#        components/Link.cpp components/Link.h
#        components/Flow.cpp components/Flow.h
#        Route.h
#        components/MyVlan.h)
add_executable(${BINARY}_run ${SOURCES})

add_library(${BINARY}_lib STATIC ${SOURCES})
target_link_libraries(${BINARY}_run
        uuid
        spdlog)