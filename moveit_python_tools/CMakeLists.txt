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

catkin_package(LIBRARIES ${MOVEIT_LIB_NAME} INCLUDE_DIRS include/)

add_library(${MOVEIT_LIB_NAME}
    src/py_conversions.cpp
    src/roscpp_initializer.cpp
)
set_target_properties(${MOVEIT_LIB_NAME} PROPERTIES VERSION ${${PROJECT_NAME}_VERSION})
target_include_directories(${MOVEIT_LIB_NAME} PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> ${roscpp_INCLUDE_DIRS} ${PYTHON_INCLUDE_DIRS})
target_link_libraries(${MOVEIT_LIB_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${PYTHON_LIBRARIES})

add_library(${MOVEIT_LIB_NAME}_python src/wrap_python_roscpp_initializer.cpp)
target_link_libraries(${MOVEIT_LIB_NAME}_python ${MOVEIT_LIB_NAME} ${PYTHON_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES})
set_target_properties(${MOVEIT_LIB_NAME}_python PROPERTIES VERSION ${${PROJECT_NAME}_VERSION})
set_target_properties(${MOVEIT_LIB_NAME}_python PROPERTIES OUTPUT_NAME _moveit_roscpp_initializer PREFIX "")
set_target_properties(${MOVEIT_LIB_NAME}_python PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_PYTHON_DESTINATION})

install(TARGETS ${MOVEIT_LIB_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(DIRECTORY include/ DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION})

install(TARGETS ${MOVEIT_LIB_NAME}_python
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION})
