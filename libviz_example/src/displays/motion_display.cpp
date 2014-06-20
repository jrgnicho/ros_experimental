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
			"robot description parameter",display_motion_category_,
			SLOT(load_robot_model_handler()),this);

	robot_alpha_property_ = new rviz::FloatProperty("Robot alpha",1,"Robot alpha",
			display_motion_category_,SLOT(update_robot_alpha_handler()),this);

	display_time_property_ = new rviz::FloatProperty("Display time",2.0f,"Display time",display_motion_category_,
			NULL,NULL);

	display_traj_topic_property_ = new rviz::RosTopicProperty("Display Trajectory Topic","trajectory_preview",
			ros::message_traits::datatype<moveit_msgs::DisplayTrajectory>(),"The topic for displaying the robot trajectory",
			display_motion_category_,SLOT(update_topic_handler() ),this);

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
	display_robot_traj_ptr_->clear();
	display_robot_traj_ptr_->setVisualVisible(true);
	display_robot_traj_ptr_->setCollisionVisible(false);
	display_robot_traj_ptr_->setVisible(false);

	if(planning_scene_render_ptr_)
		planning_scene_render_ptr_->getGeometryNode()->setVisible(false);

	rviz::Display::reset();
}

void MotionDisplay::onInitialize()
{
	using namespace moveit_rviz_plugin;

	rviz::Display::onInitialize();

	planning_scene_node_ptr_ = scene_node_->createChildSceneNode("WorldSceneNode");
	display_robot_traj_ptr_.reset(new RobotStateVisualization(planning_scene_node_ptr_,context_,
			"Robot Motion Plan",display_motion_category_));

	// payload color
	std_msgs::ColorRGBA payload_color;
	payload_color.a = 1.0f; payload_color.r = 0.0f; payload_color.g = 0.0f; payload_color.b = 1.0f;
	display_robot_traj_ptr_->setDefaultAttachedObjectColor(payload_color);

	// qt slot signal connections
	connect(this,SIGNAL(animate_robot_trajectory()),this,SLOT(animate_robot_trajectory_handler()));

	// calling handlers
	update_topic_handler();
	update_robot_alpha_handler();
	load_robot_model_handler();
}

void MotionDisplay::update_topic_handler()
{
	display_traj_subs_.shutdown();
	display_traj_subs_ = nh_.subscribe(display_traj_topic_property_->getStdString(),1,
			&MotionDisplay::process_message_callback,this);
}

void MotionDisplay::update_robot_alpha_handler()
{
	display_robot_traj_ptr_->setAlpha(robot_alpha_property_->getFloat());
}

void MotionDisplay::onEnable()
{
	rviz::Display::onEnable();
	display_robot_traj_ptr_->setVisualVisible(true);
	update_topic_handler();
	load_robot_model_handler();
}

void MotionDisplay::onDisable()
{
	display_robot_traj_ptr_->setVisualVisible(false);

	rviz::Display::onDisable();
}

void MotionDisplay::fixedFrameChanged()
{
	rviz::Display::fixedFrameChanged();
}

void MotionDisplay::animate_robot_trajectory_handler()
{
	if(way_point_index_ < robot_traj_ptr_->getWayPointCount())
	{
		ROS_INFO_STREAM("Animating way point "<<way_point_index_);

		display_robot_traj_ptr_->update(robot_traj_ptr_->getWayPointPtr(way_point_index_));
		ros::Duration(way_point_time_step_).sleep();
		way_point_index_++;

		planning_scene_render_ptr_->renderPlanningScene(planning_scene_ptr_,environment_color,attached_color,
				moveit_rviz_plugin::OCTOMAP_FREE_VOXELS,moveit_rviz_plugin::OCTOMAP_Z_AXIS_COLOR,robot_alpha_property_->getFloat());

		Q_EMIT animate_robot_trajectory();
	}
}

void MotionDisplay::load_robot_model_handler()
{
	using namespace moveit_rviz_plugin;

	robot_model_loader::RobotModelLoader robot_model_loader(robot_description_property_->getStdString());
	robot_model_ptr_ = robot_model_loader.getModel();

	display_robot_traj_ptr_->clear();
	display_robot_traj_ptr_->load(*robot_model_ptr_->getURDF());
	display_robot_traj_ptr_->setVisualVisible(true);
	display_robot_traj_ptr_->setCollisionVisible(false);
	display_robot_traj_ptr_->setVisible(false);

	planning_scene_ptr_.reset(new planning_scene::PlanningScene(robot_model_ptr_));
	planning_scene_render_ptr_.reset(new PlanningSceneRender(planning_scene_node_ptr_,context_,display_robot_traj_ptr_));
	planning_scene_render_ptr_->getGeometryNode()->setVisible(true);
}

void MotionDisplay::process_message_callback(const moveit_msgs::DisplayTrajectory::ConstPtr &msg)
{


	// setting up trajectory
	robot_traj_ptr_.reset(new robot_trajectory::RobotTrajectory(robot_model_ptr_,""));
	std::vector<moveit_msgs::RobotTrajectory>::const_iterator t;
	for(t = msg->trajectory.begin();t != msg->trajectory.end();t++)
	{
		robot_trajectory::RobotTrajectory traj(robot_model_ptr_,"");
		traj.setRobotTrajectoryMsg(planning_scene_ptr_->getCurrentState(),msg->trajectory_start,*t);
		robot_traj_ptr_->append(traj,0.0f);
	}

	// resetting animation variables
	way_point_index_ = 0;
	way_point_time_step_ = display_time_property_->getFloat()/robot_traj_ptr_->getWayPointCount();

	ROS_INFO_STREAM("moveit_msgs::DisplayTrajectory message received with "<<robot_traj_ptr_->getWayPointCount()<<" points");

	Q_EMIT animate_robot_trajectory();

}

PLUGINLIB_EXPORT_CLASS(libviz_example::displays::MotionDisplay,rviz::Display);

}}
