cmake_minimum_required(VERSION 2.8.12)
project(moveit_python_tools)

add_compile_options(-std=c++11)

if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(MOVEIT_LIB_NAME moveit_python_tools)

find_package(Boost REQUIRED COMPONENTS python)
find_package(PythonLibs 2.7 REQUIRED)
find_package(catkin REQUIRED COMPONENTS roscpp)

catkin_package(INCLUDE_DIRS include/)

add_library(${MOVEIT_LIB_NAME} SHARED
    src/py_conversions.cpp
)
target_include_directories(${MOVEIT_LIB_NAME} PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> ${roscpp_INCLUDE_DIRS} ${PYTHON_INCLUDE_DIRS})

install(DIRECTORY include/ DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION})
