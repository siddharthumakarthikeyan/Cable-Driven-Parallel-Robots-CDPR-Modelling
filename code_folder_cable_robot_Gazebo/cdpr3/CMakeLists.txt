cmake_minimum_required(VERSION 2.8.3)
project(cdpr3)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  roscpp
  roslib
  sensor_msgs
  geometry_msgs
qpOASES
  rospy
  rqt_gui
  rqt_gui_py
std_msgs
  message_generation
)

catkin_python_setup()

## System dependencies are found with CMake's conventions
find_package(gazebo REQUIRED)
find_package(VISP REQUIRED)
find_package(qpOASES REQUIRED)
find_package(CGAL REQUIRED COMPONENTS thread)
find_package(Eigen3)
if(NOT EIGEN3_FOUND)
  # Fallback to cmake_modules
  find_package(cmake_modules REQUIRED)
  find_package(Eigen REQUIRED)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
else()
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()

find_library(CGAL_LIBRARY CGAL /usr/lib)
find_library(GMP_LIBRARY gmp /usr/lib)


###################################
## catkin specific configuration ##
###################################
catkin_package(
INCLUDE_DIRS include ${VISP_INCLUDE_DIRS} ${qpOASES_INCLUDE_DIRS} ${CGAL_INCLUDE_DIRS} ${GMP_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS}
LIBRARIES ${PROJECT_NAME} ${VISP_LIBRARIES} ${GAZEBO_LIBRARIES} cdpr3 qpOASES ${CGAL_LIBRARIES} ${GMP_LIBRARIES} ${EIGEN3_LIBRARIES}
CATKIN_DEPENDS roscpp roslib gazebo_ros sensor_msgs geometry_msgs qpOASES 
 DEPENDS cgal gmp
)

#add_dependencies(${PROJECT_NAME}_gencfg ${catkin_EXPORTED_TARGETS})





###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# TODO: Check names of system library include directories (visp)
link_directories(${GAZEBO_LIBRARY_DIRS}  ${qpOASES_LIBRARY_DIRS} ${CGAL_LIBRARY_DIRS} ${GMP_LIBRARY_DIRS}  )
include_directories(include ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS} ${VISP_INCLUDE_DIRS} ${SDFormat_INCLUDE_DIRS} ${gazebo_ros_INCLUDE_DIRS} ${cdpr3_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS}  ${qpOASES_INCLUDE_DIRS}  ${CGAL_INCLUDE_DIRS} ${GMP_INCLUDE_DIRS} )


add_executable(cdpr3_control src/control.cpp src/trajectory.cpp)
target_link_libraries(cdpr3_control ${catkin_LIBRARIES} ${VISP_LIBRARIES} ${CGAL_LIBRARIES} ${EIGEN3_LIBRARIES})

add_executable(cdprnormal src/controlNormal.cpp src/trajectory.cpp)
target_link_libraries(cdprnormal ${catkin_LIBRARIES} ${VISP_LIBRARIES} ${CGAL_LIBRARIES} ${EIGEN3_LIBRARIES})

add_executable(cdprscurve src/controlsCurve.cpp src/trajectory.cpp)
target_link_libraries(cdprscurve ${catkin_LIBRARIES} ${VISP_LIBRARIES} ${CGAL_LIBRARIES} ${EIGEN3_LIBRARIES})


#add_executable(cdpr3_trajectory src/trajectory.cpp )

#add_executable(cddpr3 src/control.cpp src/trajectory.cpp)
#target_link_libraries(cddpr3 ${catkin_LIBRARIES} ${VISP_LIBRARIES} ${CGAL_LIBRARIES})

add_library(cdpr3 src/cdpr_plugin.cpp)
target_link_libraries(cdpr3  ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES} ${VISP_LIBRARIES} ${CGAL_LIBRARIES})

#target_link_libraries(cdpr3_control ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${VISP_LIBRARIES} ${CGAL_LIBRARIES})
#target_link_libraries(cdpr3_trajectory ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${VISP_LIBRARIES})





#add_library(cdpr4 src/trajectory.cpp)
#target_link_libraries(cdpr4  ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES} ${VISP_LIBRARIES})

#add_library(cdpr_plugin src/cdpr_plugin.cpp include/cdpr/cdpr_plugin.h include/freefloating_gazebo/freefloating_gazebo_control.h)
#target_link_libraries( CableDrivenParallelRobotPlugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ModelPlugin )

#install(PROGRAMS scripts/cdpr3  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

set(ROS_BUILD_TYPE Debug)
set(CMAKE_BUILD_TYPE Debug)
