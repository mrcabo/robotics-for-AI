cmake_minimum_required(VERSION 2.8.3)
project(my_planner)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
genmsg
pluginlib
pcl_ros
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need


###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations


add_service_files(
  FILES
  MakePlan.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs  # Or other packages containing msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES navigation_test_planner
#  CATKIN_DEPENDS nav_core
#  DEPENDS system_lib
)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
 add_library(global_planner_lib
   src/${PROJECT_NAME}/python_planner.cpp
 )
 

add_dependencies(global_planner_lib my_planner_generate_messages_cpp)

#install (PROGRAMS
#	scripts/dijkstra_planning.py
#	DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

#install(TARGETS global_planner_lib
#	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

#install(FILES
#	global_planner_plugin.xml
	#maps/my_map.yaml
	#maps/my_map.png
	#	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
	#)
