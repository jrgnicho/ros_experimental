/*
 * MotionDisplay.h
 *
 *  Created on: Jun 18, 2014
 *      Author: ros developer 
 */

#ifndef MOTIONDISPLAY_H_
#define MOTIONDISPLAY_H_

#include <ros/ros.h>
#include <rviz/display.h>
#include <rviz/visualization_manager.h>
#include <rviz/properties/color_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/int_property.h>
#include <rviz/properties/string_property.h>
#include <rviz/properties/ros_topic_property.h>
#include <rviz/selection/selection_manager.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit/rviz_plugin_render_tools/robot_state_visualization.h>


namespace libviz_example{ namespace displays{
class MotionDisplay : public rviz::Display
{
Q_OBJECT
public:
	MotionDisplay();
	virtual ~MotionDisplay();

	virtual void load(const rviz::Config& config);
	virtual void save(rviz::Config config) const;
	virtual void update(float wall_dt, float ros_dt);
	virtual void reset();

protected:

	virtual void process_message_callback(const moveit_msgs::DisplayTrajectory::ConstPtr &msg);

protected Q_SLOTS:

	void update_topic_property();

protected:

	ros::NodeHandle nh_;
	ros::Subscriber display_traj_subs_;

	moveit_rviz_plugin::RobotStateVisualizationPtr display_robot_traj_ptr_;

	// display properties
	rviz::Property* display_motion_category_;
	rviz::RosTopicProperty* display_traj_topic_property_;
	rviz::StringProperty* robot_description_property_;
	rviz::FloatProperty* robot_alpha_property_;
	rviz::FloatProperty* display_time_property_;
};

}}

#endif /* MOTIONDISPLAY_H_ */
