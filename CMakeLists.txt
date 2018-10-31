cmake_minimum_required(VERSION 3.8)
project(rtde)
set(CMAKE_C_COMPILER /usr/bin/gcc-7)
set(CMAKE_C_COMPILER /usr/bin/g++-7)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

SET(Boost_USE_MULTITHREAD ON)
FIND_PACKAGE(Boost REQUIRED COMPONENTS system thread regex)
IF(Boost_FOUND)
    INCLUDE_DIRECTORIES(${Boost_INCLUDE_DIRS})
    LINK_DIRECTORIES(${Boost_LIBRARY_DIRS})
ENDIF(Boost_FOUND)

# Find required protobuf package
#include(FindProtobuf)
#find_package(Protobuf REQUIRED)

include_directories(/home/prier/cppystruct/include) #${PROTOBUF_INCLUDE_DIR}

set(LIB_SOURCE_FILES rtde.cpp rtde.h) # rtde_config.pb.cc rtde_config.pb.h
add_library(rtde SHARED ${LIB_SOURCE_FILES} /home/prier/cppystruct/include) # ${PROTOBUF_INCLUDE_DIRS}

set(CLIENT_SOURCE_FILES rtde_client.cpp)
add_executable(rtde_client ${CLIENT_SOURCE_FILES})
target_link_libraries(rtde_client rtde ${Boost_SYSTEM_LIBRARY} ${Boost_THREAD_LIBRARY} ${Boost_REGEX_LIBRARY}) #  ${PROTOBUF_LIBRARIES}
