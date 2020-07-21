cmake_minimum_required(VERSION 3.5)
project(ur_rtde_examples)

find_package(ur_rtde REQUIRED PATHS "../../Build/ur_rtde" "../../build/ur_rtde")

add_executable(forcemode_example forcemode_example.cpp)
target_link_libraries(forcemode_example PUBLIC ur_rtde::rtde)

add_executable(io io_example.cpp)
target_link_libraries(io PUBLIC ur_rtde::rtde)

add_executable(movej_path_with_blend movej_path_with_blend_example.cpp)
target_link_libraries(movej_path_with_blend PUBLIC ur_rtde::rtde)

add_executable(servoj servoj_example.cpp)
target_link_libraries(servoj PUBLIC ur_rtde::rtde)

add_executable(speedj speedj_example.cpp)
target_link_libraries(speedj PUBLIC ur_rtde::rtde)