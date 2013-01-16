cmake_minimum_required(VERSION 2.8.3)
project(trajopt_plugin)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS dynamic_reconfigure moveit_core moveit_ros_planning pluginlib roscpp tf)

# Hardcode trajopt paths for now
include_directories("$ENV{HOME}/research/trajopt/src")
include_directories("$ENV{HOME}/research/trajopt/ext/json/include")
include_directories("$ENV{HOME}/research/trajopt/ext/gtest/include")
include_directories("$ENV{HOME}/research/trajopt/ext/bullet/src")

link_directories("$ENV{HOME}/research/trajopt/build/lib")

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
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs  # Or other packages containing msgs
# )

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
add_executable(test_traj src/test_traj.cpp)
target_link_libraries(test_traj ${catkin_LIBRARIES} trajopt)

set(MOVEIT_LIB_NAME moveit_trajopt_interface_ros)

add_library(${MOVEIT_LIB_NAME} src/trajopt_interface_ros.cpp src/ros_rave_conversions.cpp)
target_link_libraries(${MOVEIT_LIB_NAME}
  ${catkin_LIBRARIES}
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
