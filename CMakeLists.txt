cmake_minimum_required(VERSION 3.5)
project(rtde)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -pthread")

SET(Boost_USE_MULTITHREAD ON)
FIND_PACKAGE(Boost REQUIRED COMPONENTS system thread regex filesystem)
IF(Boost_FOUND)
    INCLUDE_DIRECTORIES(${Boost_INCLUDE_DIRS})
    LINK_DIRECTORIES(${Boost_LIBRARY_DIRS})
ENDIF(Boost_FOUND)

#include_directories()

set(LIB_SOURCE_FILES rtde.cpp rtde.h rtde_utility.h dashboard_client.cpp dashboard_client.h robot_state.h script_client.cpp script_client.h rtde_control_interface.cpp rtde_control_interface.h)
add_library(rtde SHARED ${LIB_SOURCE_FILES})

set(RTDE_RECEIVE_CLIENT_SOURCE_FILES rtde_receive_client.cpp)
add_executable(rtde_receive_client ${RTDE_RECEIVE_CLIENT_SOURCE_FILES})
target_link_libraries(rtde_receive_client rtde ${Boost_FILESYSTEM_LIBRARY} ${Boost_SYSTEM_LIBRARY} ${Boost_THREAD_LIBRARY} ${Boost_REGEX_LIBRARY})

set(RTDE_SEND_CLIENT_SOURCE_FILES rtde_send_client.cpp)
add_executable(rtde_send_client ${RTDE_SEND_CLIENT_SOURCE_FILES})
target_link_libraries(rtde_send_client rtde ${Boost_FILESYSTEM_LIBRARY} ${Boost_SYSTEM_LIBRARY} ${Boost_THREAD_LIBRARY} ${Boost_REGEX_LIBRARY})
