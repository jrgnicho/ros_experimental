/*
 * polygonal_interactive_marker_node.cpp
 *
 *  Created on: Feb 4, 2014
 *      Author: ros-industrial
 */

#include <libviz_example/interactive_marker_servers/PolygonInteractiveMarkerServer.h>
#include <ros/ros.h>

using namespace libviz_example::interactive_marker_servers;
int main(int argc,char** argv)
{
	ros::init(argc,argv,"polygonal_interactive_markers_node");
	ros::NodeHandle nh;

	PolygonInteractiveMarkerServer server;
	server.run();

	ros::spin();

	ROS_INFO_STREAM("Exiting node");


	return 0;
}



