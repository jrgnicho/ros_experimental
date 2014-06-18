/*
 * MotionDisplay.cpp
 *
 *  Created on: Jun 18, 2014
 *      Author: ros developer 
 */

#include <libviz_example/displays/motion_display.h>
#include <pluginlib/class_list_macros.h>

namespace libviz_example{ namespace displays{
MotionDisplay::MotionDisplay():
	nh_()
{
	// TODO Auto-generated constructor stub
	display_motion_category_ = new rviz::Property("Motion Plan Preview",QVariant(),"",this);
	robot_description_property_ = new rviz::StringProperty("robot description","robot_description",
			"robot description parameter",display_motion_category_,NULL,NULL);
	robot_alpha_property_ = new rviz::FloatProperty("Robot alpha",1,"Robot alpha",
			display_motion_category_,NULL,NULL);
	display_time_property_ = new rviz::FloatProperty("Display time",2.0f,"Display time",display_motion_category_,
			NULL,NULL);

	display_traj_topic_property_ = new rviz::RosTopicProperty("Display Trajectory Topic","trajectory_preview",
			ros::message_traits::datatype<moveit_msgs::DisplayTrajectory>(),"The topic for displaying the robot trajectory",
			display_motion_category_,SLOT(update_topic_property() ),this);

	// ros
	display_traj_subs_ = nh_.subscribe(display_traj_topic_property_->getStdString(),1,
			&MotionDisplay::process_message_callback,this);

}

MotionDisplay::~MotionDisplay()
{

}

void MotionDisplay::load(const rviz::Config& config)
{

}

void MotionDisplay::save(rviz::Config config) const
{

}

void MotionDisplay::update(float wall_dt, float ros_dt)
{

}

void MotionDisplay::reset()
{

}

void MotionDisplay::update_topic_property()
{
	display_traj_subs_.shutdown();
	display_traj_subs_ = nh_.subscribe(display_traj_topic_property_->getStdString(),1,
			&MotionDisplay::process_message_callback,this);
}

void MotionDisplay::process_message_callback(const moveit_msgs::DisplayTrajectory::ConstPtr &msg)
{
	ROS_INFO_STREAM("moveit_msgs::DisplayTrajectory message received");
}

PLUGINLIB_EXPORT_CLASS(libviz_example::displays::MotionDisplay,rviz::Display);

}}
