/*
 * PolygonInteractiveMarkerServer.cpp
 *
 *  Created on: Feb 3, 2014
 *      Author: ros-industrial
 */

#include <libviz_example/interactive_marker_servers/PolygonInteractiveMarkerServer.h>
#include <tf/transform_datatypes.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

using namespace interactive_markers;
using namespace libviz_example::interactive_marker_servers;

const std::string PolygonInteractiveMarkerServer::WORLD_FRAME_ID = "world_frame";
const std::string PolygonInteractiveMarkerServer::MARKER_DESCRIPTION =
		"interactive polygon menu marker";
const std::string PolygonInteractiveMarkerServer::INTERACTIVE_MARKER_NAME = "InteractiveMarker";
const int PolygonInteractiveMarkerServer::NUM_MARKERS = 10;
const int PolygonInteractiveMarkerServer::MAX_TRIANGLES = 20;

PolygonInteractiveMarkerServer::PolygonInteractiveMarkerServer() {
	// TODO Auto-generated constructor stub

}

PolygonInteractiveMarkerServer::~PolygonInteractiveMarkerServer() {
	// TODO Auto-generated destructor stub
}

bool PolygonInteractiveMarkerServer::init()
{
	srand(time(NULL));
	marker_server_ptr_ = InteractiveMarkerServerPtr(new InteractiveMarkerServer("polygon_markers"));

	// creating interactive markers
	std::stringstream ss;
	std::string name;
	geometry_msgs::Pose pose;
	tf::Transform t;

	// create callback
	interactive_markers::InteractiveMarkerServer::FeedbackCallback callback(
			boost::bind(&PolygonInteractiveMarkerServer::process_marker_callback,this,_1));

	int xy_step = 2;
	int z_step = 2;
	for(int i = 0; i < NUM_MARKERS;i++)
	{
		// create name
		ss<<INTERACTIVE_MARKER_NAME<<i;
		name=ss.str();

		// create pose
		tf2Scalar x= (NUM_MARKERS - rand()%NUM_MARKERS + 1) * xy_step;
		tf2Scalar y= (NUM_MARKERS - rand()%NUM_MARKERS + 1) * xy_step;
		tf2Scalar z= (NUM_MARKERS - rand()%NUM_MARKERS + 1) * z_step;
		t = tf::Transform(tf::Quaternion::getIdentity(),tf::Vector3(x,y,z));
		tf::poseTFToMsg(t,pose);

		// create marker
		visualization_msgs::InteractiveMarker int_marker;
		create_interactive_marker(pose,name,int_marker);

		// add marker to server
		marker_server_ptr_->insert(int_marker,
				callback);

		// save name
		marker_names_.push_back(name);
	}

	marker_server_ptr_->applyChanges();

}

void PolygonInteractiveMarkerServer::run()
{
	init();
}

void PolygonInteractiveMarkerServer::process_marker_callback(
		const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
	ROS_INFO_STREAM("control "<<feedback->control_name <<" was clicked");
}

void PolygonInteractiveMarkerServer::create_interactive_marker(
		const geometry_msgs::Pose& pose,std::string name,
		visualization_msgs::InteractiveMarker &int_marker)
{
	// create triangle list marker
	visualization_msgs::Marker marker;
	create_polygon_marker(marker,rand()%MAX_TRIANGLES + 1);

	// create menu control
	visualization_msgs::InteractiveMarkerControl menu_control;
	menu_control.interaction_mode = menu_control.MENU;
	menu_control.markers.push_back(marker);
	menu_control.always_visible = true;

	// create button control
	visualization_msgs::InteractiveMarkerControl button_control;
	button_control.interaction_mode = button_control.BUTTON;
	button_control.name = "button_control";

	// fill interactive marker
	int_marker.controls.push_back(menu_control);
	int_marker.controls.push_back(button_control);
	int_marker.header.frame_id = WORLD_FRAME_ID;
	int_marker.description = MARKER_DESCRIPTION;
	int_marker.name = name;
	int_marker.pose = pose;

}

void PolygonInteractiveMarkerServer::create_polygon_marker(
		visualization_msgs::Marker& marker,int triangles)
{

	// setting marker properties
	marker.type = marker.TRIANGLE_LIST;
	marker.color.r = 0.5f;
	marker.color.g = 0.5f;
	marker.color.b = 0.8f;
	marker.color.a = 0.4f;

	// defining triangle vertices
	tf2::Vector3 p0(-0.5f,0,0);
	tf2::Vector3 p1(0.5f,0,0);
	tf2::Vector3 p2;

	for(int i = 0; i < triangles;i++)
	{
		// computing last triangle point
		p2 = (p1-p0).rotate(tf2::Vector3(0,0,1),M_PI/6.0f) + p1;

		// creating temp vector
		std::vector<tf2::Vector3> points;
		points.push_back(p0);points.push_back(p1);points.push_back(p2);

		// adding points to marker
		for(unsigned int j = 0 ; j< points.size();j++)
		{
			geometry_msgs::Point p;
			p.x = points[i].getX();p.y = points[i].getY();p.z = points[i].getZ();
			marker.points.push_back(p);
		}

		// assigning new points for next triangle
		if(rand()%2 == 0)
		{
			p0 = p1;
			p1 = p2;
		}
		else
		{
			p0 = p2;
			p1= p0;
		}
	}
}

