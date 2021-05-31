cmake_minimum_required(VERSION 2.8.3)
project(cdpr_controllers)
add_definitions(-std=c++11)

#set(qpOASES_DIR /home/derek/ros_ws/src/cdpr/qpOASES)
#find_package(qpoases REQUIRED )

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cdpr
  roscpp
  log2plot
)
 
#set(CGAL_DIR /home/derek/External_library/CGAL-4.10) 
#set(qpOASES_LIBRARY_DIRS /home/derek/ros_ws/src/cdpr/qpOASES/libs)
#set(qpOASES_DIR /home/derek/ros_ws/src/cdpr/qpOASES)
#include_directories(/home/derek/ros_ws/src/cdpr/qpOASES)

find_package(VISP REQUIRED)
#find_package(qpOASES REQUIRED COMPONENTS)
#find_package(cmake_modules REQUIRED)
#find_library(qpOASES_LIB qpoases)
#link_libraries(${qpOASES_LIB})

# include helper file
#include( ${CGAL_USE_FILE} )
find_package( Boost REQUIRED )


###################################
## catkin specific configuration ##
###################################
catkin_package(
  # "include" within the package folder is where exported headers go
  INCLUDE_DIRS include 
  LIBRARIES
  # packages that need to be present to build/run this package
  CATKIN_DEPENDS cdpr roscpp log2plot
  #DEPENDS system_lib
)


###########
## Build ##
###########


## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
    include
  ${catkin_INCLUDE_DIRS}
  ${VISP_INCLUDE_DIRS}

)

add_library(${PROJECT_NAME}  
    include/cdpr_controllers/butterworth.h
    include/cdpr_controllers/tda.h
    src/tda.cpp       
    )




# PID controller to try CTD's (Olivier's code)
add_executable(	pid_control 
    src/pid_control.cpp)
target_link_libraries(pid_control ${catkin_LIBRARIES}  ${VISP_LIBRARIES} ${PROJECT_NAME})

# basic PID controller + QP for platform control to give an example
add_executable(	qp_pid_control 
	       	src/qp_pid_control.cpp 
		include/cdpr_controllers/qp.h)
target_link_libraries(qp_pid_control ${catkin_LIBRARIES}  ${VISP_LIBRARIES})

# Feekback linearization controller(CTC) in Cartesian space for platform control to give an example
add_executable( CTC
 	 	src/CTC.cpp
	 	include/cdpr_controllers/qp.h
		)   
target_link_libraries( CTC ${catkin_LIBRARIES} ${VISP_LIBRARIES} ${PROJECT_NAME})

 
