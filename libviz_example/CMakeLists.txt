cmake_minimum_required(VERSION 2.8.3)
project(libviz_example)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy rviz visualization_msgs)
find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)

# QT setup
include(${QT_USE_FILE})
add_definitions(-DQT_NO_KEYWORDS)

# QT Meta-object compiler: this produces additional source files under 'MOC_FILES' which then 
# need to be added to any project libraries or executables that reference the originating user source file.
qt4_wrap_cpp(MOC_FILES  include/libviz_example/widgets/RvizGrid.h) 
#set(SOURCE_FILES
#  src/widgets/RvizGrid.cpp
#  src/nodes/rviz_adjustable_grid_node.cpp
#  ${MOC_FILES}
#)


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
#   visualization_msgs
# )

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
   LIBRARIES ${PROJECT_NAME}
   CATKIN_DEPENDS roscpp rospy rviz visualization_msgs
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
# add_library(libviz_example
#   src/${PROJECT_NAME}/libviz_example.cpp
# )
# adding source files produced with Qt's Meta-object compiler
add_library(${PROJECT_NAME}_widgets src/widgets/RvizGrid.cpp ${MOC_FILES}) 
target_link_libraries(${PROJECT_NAME}_widgets ${catkin_LIBRARIES} ${QT_LIBRARIES})

# adding interactive marker library
add_library(${PROJECT_NAME}_interactive_markers 
	src/interactive_marker_servers/PolygonInteractiveMarkerServer.cpp) 

## Declare a cpp executable
add_executable(rviz_grid_node src/nodes/rviz_adjustable_grid_node.cpp )
target_link_libraries(rviz_grid_node ${PROJECT_NAME}_widgets ${catkin_LIBRARIES} ${QT_LIBRARIES})

# interactive marker server node
add_executable(polygonal_interactive_marker_node src/nodes/polygonal_interactive_marker_node.cpp )
target_link_libraries(polygonal_interactive_marker_node ${PROJECT_NAME}_interactive_markers ${catkin_LIBRARIES})

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(libviz_example_node libviz_example_generate_messages_cpp)

## Specify libraries to link a library or executable target against
# target_link_libraries(libviz_example_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/groovy/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS libviz_example libviz_example_node
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_libviz_example.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
