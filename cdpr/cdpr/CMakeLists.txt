cmake_minimum_required(VERSION 2.8.3)
project(cdpr)
add_definitions(-std=c++11)
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  roscpp
  roslib
  sensor_msgs
  geometry_msgs
  message_generation
)

find_package(VISP REQUIRED)

## System dependencies are found with CMake's conventions
find_package(gazebo REQUIRED)

add_message_files(
  FILES
  Tensions.msg
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
INCLUDE_DIRS include ${VISP_INCLUDE_DIRS}
LIBRARIES ${PROJECT_NAME}
CATKIN_DEPENDS roscpp roslib gazebo_ros sensor_msgs geometry_msgs message_runtime
DEPENDS ${VISP_LIBRARIES}
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# TODO: Check names of system library include directories (visp)
link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(include ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS} ${VISP_INCLUDE_DIRS})

# small model plugin subscribes to joint efforts and applies them in Gazebo
add_library(cdpr_plugin src/cdpr_plugin.cpp include/cdpr/cdpr_plugin.h)
target_link_libraries(cdpr_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
add_dependencies(cdpr_plugin ${${PROJECT_NAME}_EXPORTED_TARGETS})

# CDPR class to interface with Gazebo
add_library(cdpr src/cdpr.cpp include/cdpr/cdpr.h)
target_link_libraries(cdpr ${catkin_LIBRARIES} ${VISP_LIBRARIES})

add_executable(param src/param.cpp)
target_link_libraries(param ${catkin_LIBRARIES} ${VISP_LIBRARIES})
