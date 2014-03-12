/*
 * PolygonInteractiveMarkerServer.cpp
 *
 *  Created on: Feb 3, 2014
 *      Author: ros-industrial
 */

#include <libviz_example/interactive_marker_servers/PolygonInteractiveMarkerServer.h>
#include <tf/transform_datatypes.h>
#include <tf/tf.h>

using namespace interactive_markers;
using namespace libviz_example::interactive_marker_servers;

const std::string PolygonInteractiveMarkerServer::WORLD_FRAME_ID = "world_frame";
const std::string PolygonInteractiveMarkerServer::MARKER_DESCRIPTION =
		"interactive marker";
const std::string PolygonInteractiveMarkerServer::INTERACTIVE_MARKER_NAME = "interactive_marker";
const int PolygonInteractiveMarkerServer::NUM_MARKERS = 10;
const int PolygonInteractiveMarkerServer::MAX_TRIANGLES = 20;

PolygonInteractiveMarkerServer::PolygonInteractiveMarkerServer() {
	// TODO Auto-generated constructor stub

}

PolygonInteractiveMarkerServer::~PolygonInteractiveMarkerServer() {
	// TODO Auto-generated destructor stub
	marker_server_ptr_.reset();
}

bool PolygonInteractiveMarkerServer::init()
{
	srand(time(NULL));
	marker_server_ptr_ = InteractiveMarkerServerPtr(
			new InteractiveMarkerServer("polygon_markers","",false));

	// creating interactive markers
	std::stringstream ss;
	std::string name;
	geometry_msgs::Pose pose;
	tf::Transform t;

	// create callbacks
	interactive_markers::InteractiveMarkerServer::FeedbackCallback button_callback(
			boost::bind(&PolygonInteractiveMarkerServer::button_marker_callback,this,_1));

	interactive_markers::InteractiveMarkerServer::FeedbackCallback menu_callback(
				boost::bind(&PolygonInteractiveMarkerServer::menu_marker_callback,this,_1));

	// setup menu handler
	menu_handler_.insert("Option 1",menu_callback);
	menu_handler_.insert("Option 2",menu_callback);
	interactive_markers::MenuHandler::EntryHandle submenu_handle = menu_handler_.insert("Submenu");
	menu_handler_.insert(submenu_handle,"Submenu Option 1",menu_callback);
	menu_handler_.insert(submenu_handle,"Submenu Option 2",menu_callback);


	double xy_step = 0.2f;
	double z_step = 0.1f;
	for(int i = 0; i < NUM_MARKERS;i++)
	{
		// create name
		ss.str("");
		ss<<INTERACTIVE_MARKER_NAME<<i;
		name=ss.str();

		// create pose
		tfScalar x= (NUM_MARKERS - rand()%NUM_MARKERS + 1) * xy_step;
		tfScalar y= (NUM_MARKERS - rand()%NUM_MARKERS + 1) * xy_step;
		tfScalar z= (NUM_MARKERS - rand()%NUM_MARKERS + 1) * z_step;
		t = tf::Transform(tf::Quaternion::getIdentity(),tf::Vector3(x,y,z));
		tf::poseTFToMsg(t,pose);

		// create marker
		visualization_msgs::InteractiveMarker int_marker;
		create_interactive_marker(pose,name,int_marker);

		// add marker to server
		marker_server_ptr_->insert(int_marker,button_callback);
		menu_handler_.apply(*marker_server_ptr_,int_marker.name);

		// save name
		marker_names_.push_back(name);
	}

	marker_server_ptr_->applyChanges();
}

void PolygonInteractiveMarkerServer::run()
{
	init();
}

void PolygonInteractiveMarkerServer::button_marker_callback(
		const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
	switch(feedback->event_type)
	{
	case visualization_msgs::InteractiveMarkerFeedback::BUTTON_CLICK:
		ROS_INFO_STREAM("marker "<<feedback->marker_name <<" button control was clicked");
		marker_server_ptr_->applyChanges();
		break;

	case visualization_msgs::InteractiveMarkerFeedback::MENU_SELECT:
		ROS_INFO_STREAM("marker "<<feedback->marker_name <<" menu control was clicked");
		marker_server_ptr_->applyChanges();
		break;
	}
}

void PolygonInteractiveMarkerServer::menu_marker_callback(
		const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
	switch(feedback->event_type)
	{
	case visualization_msgs::InteractiveMarkerFeedback::BUTTON_CLICK:
		ROS_INFO_STREAM("marker "<<feedback->marker_name <<" button control was clicked");
		marker_server_ptr_->applyChanges();
		break;

	case visualization_msgs::InteractiveMarkerFeedback::MENU_SELECT:
		ROS_INFO_STREAM("marker "<<feedback->marker_name <<" menu control was clicked");
		marker_server_ptr_->applyChanges();
		break;
	}
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
	menu_control.description = "Menu " + name;
	menu_control.name = std::string("menu_") + name;

	// create button control
	visualization_msgs::InteractiveMarkerControl button_control;
	button_control.interaction_mode = button_control.BUTTON;
	button_control.markers.push_back(marker);
	button_control.name = "button_" + name;
	button_control.always_visible = true;

	// fill interactive marker
	//int_marker.controls.push_back(menu_control);
	int_marker.controls.push_back(button_control);
	int_marker.scale = 1;
	int_marker.header.frame_id = WORLD_FRAME_ID;
	int_marker.description = MARKER_DESCRIPTION;
	int_marker.name = name;
	int_marker.pose = pose;

}

void PolygonInteractiveMarkerServer::create_polygon_marker(
		visualization_msgs::Marker& marker,int triangles)
{

	ROS_INFO_STREAM("Creating polygon of "<<triangles<<" triangles");

	// setting marker properties
	marker.type = marker.TRIANGLE_LIST;
	marker.scale.x = marker.scale.y = marker.scale.z = 1;
/*	marker.header.frame_id = WORLD_FRAME_ID;*/
	marker.pose.position.x = marker.pose.position.y = marker.pose.position.z = 0;
	marker.pose.orientation.x = marker.pose.orientation.y = marker.pose.orientation.z =0;
	marker.pose.orientation.w = 1;
	marker.color.r = 0.5f;
	marker.color.g = 0.5f;
	marker.color.b = 0.8f;
	marker.color.a = 0.4f;

	// defining triangle vertices
	tf::Vector3 p0(-0.05f,0,0);
	tf::Vector3 p1(0.05f,0,0);
	tf::Vector3 p2;

	// rotation transform
	tf::Transform rot(tf::Quaternion(tf::Vector3(0,0,1),M_PI/3.0f),tf::Vector3(0,0,0));

	for(int i = 0; i < triangles;i++)
	{
		// computing last triangle point
		p2 = rot*(p1-p0) + p0;
		//p2 = (p1-p0).rotate(tf2::Vector3(0,0,1),M_PI/6.0f) + p1;

		// creating temp vector
		std::vector<tf::Vector3> points;
		points.push_back(p0);points.push_back(p1);points.push_back(p2);

		// adding points to marker
		for(unsigned int j = 0 ; j< points.size();j++)
		{
			geometry_msgs::Point p;
			p.x = points[j].getX();p.y = points[j].getY();p.z = points[j].getZ();
			marker.points.push_back(p);
		}

		// assigning new points for next triangle
		if(rand()%2 == 0)
		{
			p1 = p2;
		}
		else
		{
			p0 = p2;
		}
	}
}

