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
#include <moveit/rviz_plugin_render_tools/planning_scene_render.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/planning_scene/planning_scene.h>


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

	// overrides from Display
	virtual void onInitialize();
	virtual void onEnable();
	virtual void onDisable();
	virtual void fixedFrameChanged();

	virtual void process_message_callback(const moveit_msgs::DisplayTrajectory::ConstPtr &msg);



protected Q_SLOTS:

	void update_topic_handler();
	void update_robot_alpha_handler();
	void animate_robot_trajectory_handler();
	void load_robot_model_handler();

Q_SIGNALS:
	void animate_robot_trajectory();

protected:

	ros::NodeHandle nh_;
	ros::Subscriber display_traj_subs_;


	// display properties
	rviz::Property* display_motion_category_;
	rviz::RosTopicProperty* display_traj_topic_property_;
	rviz::StringProperty* robot_description_property_;
	rviz::FloatProperty* robot_alpha_property_;
	rviz::FloatProperty* display_time_property_;

	// scene components
	Ogre::SceneNode* planning_scene_node_ptr_;
	robot_model::RobotModelPtr robot_model_ptr_;
	planning_scene::PlanningScenePtr planning_scene_ptr_;
	moveit_rviz_plugin::PlanningSceneRenderPtr planning_scene_render_ptr_;
	rviz::Color attached_color;
	rviz::Color environment_color;

	// motion plan animation
	int way_point_index_;
	float way_point_time_step_;
	robot_trajectory::RobotTrajectoryPtr robot_traj_ptr_;
	moveit_rviz_plugin::RobotStateVisualizationPtr display_robot_traj_ptr_;
};

}}

#endif /* MOTIONDISPLAY_H_ */
