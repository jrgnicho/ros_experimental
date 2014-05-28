cmake_minimum_required(VERSION 2.8.3)
project(handle_detection_test)
find_package(catkin REQUIRED COMPONENTS roscpp pcl pcl_ros sensor_msgs visualization_msgs)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)

###################################
## catkin specific configuration ##
###################################
catkin_package(
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  ${catkin_INCLUDE_DIRS}
)

#############
## Install ##
#############

