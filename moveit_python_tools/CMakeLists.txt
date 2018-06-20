cmake_minimum_required(VERSION 2.8.12)
project(moveit_python_tools)

set(MOVEIT_LIB_NAME moveit_python_tools)

find_package(Boost REQUIRED COMPONENTS python)

find_package(catkin REQUIRED)

catkin_package(INCLUDE_DIRS include/)

add_library(${MOVEIT_LIB_NAME} INTERFACE)
target_include_directories(${MOVEIT_LIB_NAME} INTERFACE include/)

install(DIRECTORY include/ DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION})
