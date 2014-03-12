cmake_minimum_required(VERSION 2.8.3)
project(pick_and_place)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS actionlib moveit_msgs moveit_ros_manipulation moveit_ros_move_group 
moveit_ros_planning_interface tf roscpp genmsg message_generation)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/groovy/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

#######################################
## Declare ROS messages and services ##
#######################################

## Generate messages in the 'msg' folder
add_message_files(
   FILES
   GraspCommand.msg
 )

## Generate services in the 'srv' folder
add_service_files(
   FILES
   GraspService.srv
 )

## Generate added messages and services with any dependencies listed here
generate_messages(
	DEPENDENCIES
	std_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
   INCLUDE_DIRS include
#  LIBRARIES pick_and_place
#  CATKIN_DEPENDS actionlib industrial_robot_simulator moveit_msgs moveit_ros_manipulation moveit_ros_move_group  roscpp
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp library
# add_library(pick_and_place
#   src/${PROJECT_NAME}/pick_and_place.cpp
# )

## Declare a cpp executable
add_executable(pick_and_place_node src/nodes/pick_and_place_node.cpp)
add_dependencies(pick_and_place_node pick_and_place_generate_messages_cpp)
target_link_libraries(pick_and_place_node ${catkin_LIBRARIES})

add_executable(grasp_service src/nodes/grasp_service_server.cpp)
add_dependencies(grasp_service pick_and_place_generate_messages_cpp)
target_link_libraries(grasp_service ${catkin_LIBRARIES})


#############
## Install ##
#############


#############
## Testing ##
#############

