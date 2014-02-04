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

	PolygonInteractiveMarkerServer server;
	server.run();

	ros::Duration loop_duration(0.2f);
	while(ros::ok())
	{
		ros::spinOnce();
		loop_duration.sleep();
	}

	return 0;
}



