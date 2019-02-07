cmake_minimum_required(VERSION 3.5)
list(INSERT CMAKE_MODULE_PATH 0 "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

# options
option(CODE "Build primary libraries and executables." ON)
if(NOT DEFINED DOC)
	set(DOC OFF CACHE STRING
			"The documentation type to generate." FORCE)
endif()

# disable base languages
unset(PROJECT_LANGUAGES)

if(${CODE})
	set(PROJECT_LANGUAGES ${PROJECT_LANGUAGES} C CXX)
	set(CMAKE_CXX_STANDARD 11)
	set(CMAKE_CXX_STANDARD_REQUIRED ON)
	set(CMAKE_CXX_EXTENSIONS OFF)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -pthread")
endif()

project(ur_rtde VERSION 1.0.0 LANGUAGES ${PROJECT_LANGUAGES})
set(PROJECT_VERSION_SUFFIX "") # alpha/beta/rc, e.g. "-rc0"
set(PROJECT_VERSION "${PROJECT_VERSION}${PROJECT_VERSION_SUFFIX}")
set(PROJECT_AUTHOR "Anders Prier Lindvig")
set(PROJECT_COPYRIGHT "2019")
set(PROJECT_MAIL "anpl@mmmi.sdu.dk")

if(${CODE})
	set(Boost_USE_MULTITHREAD ON)
	find_package(Boost REQUIRED COMPONENTS system thread regex filesystem)
	if(Boost_FOUND)
	    include_directories(${Boost_INCLUDE_DIRS})
	    link_directories(${Boost_LIBRARY_DIRS})
	endif(Boost_FOUND)

	add_subdirectory(pybind11)
endif()

set(LIB_SOURCE_FILES
        src/rtde.cpp
        src/dashboard_client.cpp
        src/script_client.cpp
        src/rtde_control_interface.cpp
		src/rtde_receive_interface.cpp)

set(LIB_HEADER_FILES
        include/rtde.h
        include/rtde_utility.h
        include/dashboard_client.h
        include/robot_state.h
        include/script_client.h
        include/rtde_control_interface.h
		include/rtde_receive_interface.h)

include_directories(
        include
)

if(NOT DOC STREQUAL "OFF")
	add_subdirectory(doc)
endif()

if(${CODE})

	# Write the rtde_control script to a header file
	file(READ scripts/rtde_control.script RTDE_CONTROL_SCRIPT)
	set(header "const std::string NEW_LINE= \"\\n\"; const std::string QUOTATION = \"\\\"\"; std::string UR_SCRIPT = \"")
	string(REGEX REPLACE "\"" "\"+QUOTATION+\"" FILEVAR ${RTDE_CONTROL_SCRIPT})
	string(REGEX REPLACE "\\\n" "\" + NEW_LINE  + \n\"" FILEVAR ${FILEVAR})
	set(end "\";")
	file(WRITE include/rtde_control_script.h "${header}${FILEVAR}${end}")

	add_library(rtde SHARED ${LIB_SOURCE_FILES} ${LIB_HEADER_FILES})
	target_include_directories(rtde
			PRIVATE ${PYBIND11_INCLUDE_DIR}  # from project CMakeLists.txt
			PRIVATE ${pybind11_INCLUDE_DIR}  # from pybind11Config
			PRIVATE ${PYTHON_INCLUDE_DIRS})
	target_link_libraries(rtde ${PYTHON_LIBRARIES})

	pybind11_add_module(rtde_control SHARED ${LIB_SOURCE_FILES} ${LIB_HEADER_FILES})
	target_link_libraries(rtde_control PRIVATE ${Boost_FILESYSTEM_LIBRARY} ${Boost_SYSTEM_LIBRARY} ${Boost_THREAD_LIBRARY} ${Boost_REGEX_LIBRARY})

	pybind11_add_module(rtde_receive SHARED ${LIB_SOURCE_FILES} ${LIB_HEADER_FILES})
	target_link_libraries(rtde_receive PRIVATE ${Boost_FILESYSTEM_LIBRARY} ${Boost_SYSTEM_LIBRARY} ${Boost_THREAD_LIBRARY} ${Boost_REGEX_LIBRARY})

	set(RTDE_RECEIVE_CLIENT_SOURCE_FILES src/rtde_receive_client.cpp)
	add_executable(rtde_receive_client ${RTDE_RECEIVE_CLIENT_SOURCE_FILES})
	target_include_directories(rtde_receive_client
			PRIVATE ${PYBIND11_INCLUDE_DIR}  # from project CMakeLists.txt
			PRIVATE ${pybind11_INCLUDE_DIR}  # from pybind11Config
			PRIVATE ${PYTHON_INCLUDE_DIRS})
	target_link_libraries(rtde_receive_client PRIVATE rtde ${Boost_FILESYSTEM_LIBRARY} ${Boost_SYSTEM_LIBRARY} ${Boost_THREAD_LIBRARY} ${Boost_REGEX_LIBRARY})

	set(RTDE_SEND_CLIENT_SOURCE_FILES src/rtde_send_client.cpp)
	add_executable(rtde_send_client ${RTDE_SEND_CLIENT_SOURCE_FILES})
	target_include_directories(rtde_send_client
			PRIVATE ${PYBIND11_INCLUDE_DIR}  # from project CMakeLists.txt
			PRIVATE ${pybind11_INCLUDE_DIR}  # from pybind11Config
			PRIVATE ${PYTHON_INCLUDE_DIRS})
	target_link_libraries(rtde_send_client PRIVATE rtde ${Boost_FILESYSTEM_LIBRARY} ${Boost_SYSTEM_LIBRARY} ${Boost_THREAD_LIBRARY} ${Boost_REGEX_LIBRARY})
endif()
