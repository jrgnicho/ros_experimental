cmake_minimum_required(VERSION 2.8.3)
project(libviz_example)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS  interactive_markers rviz genmsg tf roscpp rospy rviz visualization_msgs moveit_msgs moveit_ros_planning_interface moveit_ros_visualization)
find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)

# QT setup
include(${QT_USE_FILE})
add_definitions(-DQT_NO_KEYWORDS)

# QT Meta-object compiler: this produces additional source files under 'MOC_FILES' which then 
# need to be added to any project libraries or executables that reference the originating user source file.
qt4_wrap_cpp(MOC_FILES  
	include/libviz_example/widgets/RvizGrid.h
	include/libviz_example/panels/TestRvizPanel.h
	include/libviz_example/panels/help_panel.h
	include/libviz_example/displays/motion_display.h) 
#set(SOURCE_FILES
#  src/widgets/RvizGrid.cpp
#  src/nodes/rviz_adjustable_grid_node.cpp
#  ${MOC_FILES}
#)


## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)

###################################
## catkin specific configuration ##
###################################
catkin_package(
   INCLUDE_DIRS include
   LIBRARIES ${PROJECT_NAME}
   CATKIN_DEPENDS interactive_markers rviz tf roscpp rospy rviz visualization_msgs moveit_msgs moveit_ros_planning_interface moveit_ros_visualization
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include
  ${catkin_INCLUDE_DIRS}
)
link_directories(${catkin_LIBRARY_DIRS})

 
# adding source files produced with Qt's Meta-object compiler
add_library(${PROJECT_NAME} 
	src/widgets/RvizGrid.cpp 
	src/interactive_marker_servers/PolygonInteractiveMarkerServer.cpp
	src/panels/TestRvizPanel.cpp
	src/panels/help_panel.cpp
	src/displays/motion_display.cpp
	${MOC_FILES}) 
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${QT_LIBRARIES})

## Declare a cpp executable
add_executable(rviz_grid_node src/nodes/rviz_adjustable_grid_node.cpp )
target_link_libraries(rviz_grid_node ${PROJECT_NAME} ${catkin_LIBRARIES} ${QT_LIBRARIES})

# interactive marker server node
add_executable(polygonal_interactive_marker_node src/nodes/polygonal_interactive_marker_node.cpp )
target_link_libraries(polygonal_interactive_marker_node ${PROJECT_NAME} ${catkin_LIBRARIES})


## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(libviz_example_node libviz_example_generate_messages_cpp)

## Specify libraries to link a library or executable target against
# target_link_libraries(libviz_example_node
#   ${catkin_LIBRARIES}
# )

## Install rules

install(TARGETS
  ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES 
  plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY icons/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/icons)

