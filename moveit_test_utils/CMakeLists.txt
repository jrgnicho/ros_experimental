cmake_minimum_required(VERSION 2.8.3)
project(moveit_test_utils)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  roscpp
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

###################################
## catkin specific configuration ##
###################################
catkin_package(
)

###########
## Build ##
###########

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(moveit_goal_commander src/moveit_goal_commander.cpp)
target_link_libraries(moveit_goal_commander ${catkin_LIBRARIES})

#############
## Install ##
#############

#############
## Testing ##
#############

