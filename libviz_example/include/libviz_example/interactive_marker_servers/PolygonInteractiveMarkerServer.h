/*
 * PolygonInteractiveMarkerServer.h
 *
 *  Created on: Feb 3, 2014
 *      Author: ros-industrial
 */

#ifndef POLYGONINTERACTIVEMARKERSERVER_H_
#define POLYGONINTERACTIVEMARKERSERVER_H_

#include <ros/ros.h>
#include <interactive_markers/interactive_marker_server.h>

namespace interactive_markers
{
	typedef boost::shared_ptr<interactive_markers::InteractiveMarkerServer> InteractiveMarkerServerPtr;
}

namespace libviz_example{ namespace interactive_marker_servers{

class PolygonInteractiveMarkerServer {
public:
	PolygonInteractiveMarkerServer();
	virtual ~PolygonInteractiveMarkerServer();

	void run();

public:

	static const std::string WORLD_FRAME_ID;
	static const std::string MARKER_DESCRIPTION;
	static const std::string INTERACTIVE_MARKER_NAME;
	static const int NUM_MARKERS;
	static const int MAX_TRIANGLES;


protected:

	interactive_markers::InteractiveMarkerServerPtr marker_server_ptr_;
	std::vector<std::string> marker_names_;

protected:


	bool init();
	void process_marker_callback(
			const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
	void create_polygon_marker(visualization_msgs::Marker& marker,int triangles);
	void create_interactive_marker(const geometry_msgs::Pose& pose,std::string name,
			visualization_msgs::InteractiveMarker &int_marker);

};
}}

#endif /* POLYGONINTERACTIVEMARKERSERVER_H_ */
