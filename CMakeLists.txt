cmake_minimum_required(VERSION 2.8.3)
project(trajopt_plugin)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS dynamic_reconfigure moveit_core moveit_ros_planning pluginlib roscpp tf)

# Do something reasonably useful to find trajopt
set(TRAJOPT_HOME "" CACHE STRING "Trajopt directory (e.g. ~/trajopt)")
if ("${TRAJOPT_HOME}" STREQUAL "")
  message(STATUS "Attempting to find trajopt")
  find_path(problem_description src/trajopt/problem_description.hpp PATHS ~/research/trajopt ~/trajopt )
  string(FIND ${problem_description} "src/trajopt" pathend REVERSE)
  string(SUBSTRING ${problem_description} 0 ${pathend} TRAJOPT_HOME)
  message("Trajopt path: ${TRAJOPT_HOME}")
else()
  message(STATUS "Using trajopt from ${TRAJOPT_HOME}")
endif()

# Hardcode trajopt paths for now
include_directories("${TRAJOPT_HOME}/src")
include_directories("${TRAJOPT_HOME}/ext/json/include")
include_directories("${TRAJOPT_HOME}/ext/gtest/include")
include_directories("${TRAJOPT_HOME}/ext/bullet/src")

link_directories("${TRAJOPT_HOME}/build/lib")

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

find_package(OpenRAVE 0.9 REQUIRED)

include_directories( ${OpenRAVE_INCLUDE_DIRS})
## Uncomment this if the package has a setup.py. This macro ensures
## modules and scripts declared therein get installed
# catkin_python_setup()

#######################################
## Declare ROS messages and services ##
#######################################

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
add_service_files(
  DIRECTORY srv
)

generate_messages(
  DEPENDENCIES
  std_msgs)

###################################################
## Declare things to be passed to other projects ##
###################################################

## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
   INCLUDE_DIRS include
#  LIBRARIES trajopt_plugin
#  CATKIN_DEPENDS dynamic_reconfigure moveit_core moveit_ros_planning pluginlib roscpp tf
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(include
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp library
# add_library(trajopt_plugin
#   src/${PROJECT_NAME}/trajopt_plugin.cpp
# )

## Declare a cpp executable

set(MOVEIT_LIB_NAME moveit_trajopt_interface_ros)

add_library(${MOVEIT_LIB_NAME} src/trajopt_interface_ros.cpp src/ros_rave_conversions.cpp)
target_link_libraries(${MOVEIT_LIB_NAME}
  ${catkin_LIBRARIES}
  trajopt
  ${OpenRAVE_LIBRARIES}
  ${OpenRAVE_CORE_LIBRARIES}
  )

add_library(moveit_trajopt_planner_plugin src/trajopt_plugin.cpp)
target_link_libraries(moveit_trajopt_planner_plugin
  ${MOVEIT_LIB_NAME}
  ${catkin_LIBRARIES}
  ${OpenRAVE_LIBRARIES}
  ${OpenRAVE_CORE_LIBRARIES}
  trajopt
  )

## Add dependencies to the executable
# add_dependencies(trajopt_plugin_node ${PROJECT_NAME})

## Specify libraries to link a library or executable target against
# target_link_libraries(trajopt_plugin_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

## Mark executable scripts (Python etc.) for installation
## not required for python when using catkin_python_setup()
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS trajopt_plugin trajopt_plugin_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_trajopt_plugin.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
