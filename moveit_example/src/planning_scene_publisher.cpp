/*
 * planning_scene_publisher.cpp
 *
 *  Created on: Apr 25, 2013
 *      Author: ros developer 
 */

#include <string.h>

#include <ros/ros.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/Pose.h>
#include <tf/transform_datatypes.h>

// moveit
#include <moveit/move_group_interface/move_group.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>

#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/GetStateValidity.h>
#include <moveit_msgs/DisplayRobotState.h>

// boost
#include <boost/assign.hpp>
#include <boost/foreach.hpp>
#include <boost/assign/list_of.hpp>

// topics
static const std::string ATTACHED_COLLISION_OBJECT_TOPIC = "attached_collision_object";
static const std::string COLLISION_OBJECT_TOPIC = "collision_object";
static const std::string PLANNING_SCENE_DIFF_TOPIC = "planning_scene";

// parameters
static std::string COLLISION_OBJECT_PARAM = "collision_object_info";
static std::string ATTACHED_OBJECT_PARAM = "attached_object_info";
static std::string ARM_GROUP_PARAM = "arm_group";
static std::string TCP_LINK_PARAM = "tcp_link";

// global names
static const std::string ATTACHED_OBJECT_ID = "attached_obj";
static const std::string COLLISION_OBJECT_ID = "collision_obj";

// constants
static const tf::Vector3 BOX_POSITION = tf::Vector3(0.0f,0.0f,0.0f);
static const double BOX_SIDE_LENGTH = 0.1f;


using namespace boost::assign;

namespace command_flags
{
	enum CommandFlags
	{
		EXIT = -1,
		ADD_COLLISION_OBJECT = 0,
		ATTACHED_OBJECT_TO_TCP = 1,
		UPDATE_COLLISION_OBJECT = 2,
		REMOVE_COLLISION_OBJECT = 3,
		DETACH_OBJECT_FROM_TCP = 4,
		ADD_OBJECT_TO_PLANNING_SCENE = 5,
		ATTACH_TO_TCP_IN_PLANNING_SCENE = 6,
		UPDATE_OBJECT_IN_PLANNING_SCENE = 7,
		REMOVE_OBJECT_FROM_PLANNING_SCENE = 8,
		DETACH_FROM_TCP_IN_PLANNING_SCENE = 9,
		MOVE_ARM_TO_RANDOM_TARGET = 10,
		NONE = 11
	};

	static std::map<int,std::string> NAME_MAPPINGS = map_list_of((int)ADD_COLLISION_OBJECT,"ADD COLLISION_OBJECT")
		((int)ATTACHED_OBJECT_TO_TCP,"ATTACHED_OBJECT_TO_TCP")
		((int)UPDATE_COLLISION_OBJECT,"UPDATE_COLLISION_OBJECT")
		((int)REMOVE_COLLISION_OBJECT,"REMOVE_COLLISION_OBJECT")
		((int)DETACH_OBJECT_FROM_TCP,"DETACH_OBJECT_FROM_TCP")
		((int)ADD_OBJECT_TO_PLANNING_SCENE,"ADD_OBJECT_TO_PLANNING_SCENE")
		((int)ATTACH_TO_TCP_IN_PLANNING_SCENE,"ATTACH_TO_TCP_IN_PLANNING_SCENE")
		((int)UPDATE_OBJECT_IN_PLANNING_SCENE,"UPDATE_OBJECT_IN_PLANNING_SCENE")
		((int)REMOVE_OBJECT_FROM_PLANNING_SCENE,"REMOVE_OBJECT_FROM_PLANNING_SCENE")
		((int)DETACH_FROM_TCP_IN_PLANNING_SCENE,"DETACH_FROM_TCP_IN_PLANNING_SCENE")
		((int)MOVE_ARM_TO_RANDOM_TARGET,"MOVE_ARM_TO_RANDOM_TARGET")
		((int)EXIT ,"EXIT ");
}

struct CollisionObjectDetails
{

	CollisionObjectDetails()
	{

	}

	geometry_msgs::Pose pose_;
	std::string id_;
	std::string frame_id_;
	std::string link_name_; // name of arm link if attached to robot
	tf::Vector3 size_;
	int shape_type_;


	bool fetch_parameters(std::string name_space)
	{
		ros::NodeHandle nh("~");
		bool success = true;
		XmlRpc::XmlRpcValue param;
		shape_msgs::SolidPrimitive shape;
		tf::Vector3 axis;
		double val;

		success = nh.getParam(name_space + "/frame_id",frame_id_) && nh.getParam(name_space + "/id",id_) &&
				nh.getParam(name_space + "/link_name",link_name_) && nh.getParam(name_space + "/shape_type",shape_type_);


		if(success && nh.getParam(name_space + "/size",param))
		{
			val = static_cast<double>(param["x"]); size_.setX(val);
			val = static_cast<double>(param["y"]); size_.setY(val);
			val = static_cast<double>(param["z"]); size_.setZ(val);
		}
		else
		{
			success = false;
		}

		if(success && nh.getParam(name_space + "/position",param))
		{
			val = static_cast<double>(param["x"]); pose_.position.x = val;
			val = static_cast<double>(param["y"]); pose_.position.y = val;
			val = static_cast<double>(param["z"]); pose_.position.z = val;
		}
		else
		{
			success = false;
		}

		if(success && nh.getParam(name_space + "/orientation",param))
		{
			val = static_cast<double>(param["x"]); pose_.orientation.x = val;
			val = static_cast<double>(param["y"]); pose_.orientation.y = val;
			val = static_cast<double>(param["z"]); pose_.orientation.z = val;
			val = static_cast<double>(param["z"]); pose_.orientation.w = val;
		}
		else
		{
			success = false;
		}

		if(success)
		{
			// building shape
			shape.type = shape_msgs::SolidPrimitive::BOX;
			shape.dimensions.resize(3);
			shape.dimensions[0] = size_.getX();shape.dimensions[1] = size_.getY();shape.dimensions[2] = size_.getZ();

			// filling collision object members
			collision_obj_.primitives.clear();
			collision_obj_.primitive_poses.clear();
			collision_obj_.primitives.push_back(shape);
			collision_obj_.primitive_poses.push_back(pose_);
			collision_obj_.header.frame_id = frame_id_;
			collision_obj_.id = id_;

			// filling attached object members
			attached_obj_.link_name = link_name_;
			attached_obj_.object = collision_obj_;
			attached_obj_.object.header.frame_id = link_name_;
		}

		return success;
	}

	void get_collision_object(moveit_msgs::CollisionObject &obj)
	{
		obj = collision_obj_;
	}

	void get_attached_object(moveit_msgs::AttachedCollisionObject &obj)
	{
		obj = attached_obj_;
	}

protected:

	moveit_msgs::CollisionObject collision_obj_;
	moveit_msgs::AttachedCollisionObject attached_obj_;
};

class PlanningSceneTest
{
public:

	PlanningSceneTest()
	{

	}

	virtual ~PlanningSceneTest()
	{

	}

	virtual void run()
	{
		ros::AsyncSpinner spinner(1);
		spinner.start();

		int command_entry = command_flags::NONE;
		if(setup())
		{

			while(ros::ok())
			{
				std::cout<<"\n\tCommand List:\n";
				typedef std::map<int,std::string> MapType;
				BOOST_FOREACH(MapType::value_type& p,command_flags::NAME_MAPPINGS)
				{
					std::cout<<"\t\t"<<p.first <<" - "<<p.second <<"\n";
				}

				std::cout<<"\tEnter Command: ";
				if(!(std::cin>>command_entry && update_parameters() && process_command(command_entry)))
				{
					break;
				}
			}

		}

	}

protected:

	virtual bool setup()
	{
		ros::NodeHandle nh;

		if(fetch_parameters())
		{
			ROS_INFO_STREAM("parameters were read successfully");
		}
		else
		{
			ROS_ERROR_STREAM("failed to read parameters, exiting");
			return false;
		}

		// ros publishers
		planning_scene_diff_pub_ = nh.advertise<moveit_msgs::PlanningScene>(PLANNING_SCENE_DIFF_TOPIC,1);
		collision_object_pub_ = nh.advertise<moveit_msgs::CollisionObject>(COLLISION_OBJECT_TOPIC,1);
		attached_object_pub_ = nh.advertise<moveit_msgs::AttachedCollisionObject>(ATTACHED_COLLISION_OBJECT_TOPIC,1);

		// setting up planning scene diff message
		planning_scene_msgs_.is_diff = true;

		// setting up move group interface
		move_group_ptr_ = boost::shared_ptr<move_group_interface::MoveGroup>(new move_group_interface::MoveGroup(arm_group_name_));

		// waiting for topics
		while(planning_scene_diff_pub_.getNumSubscribers() == 0 || collision_object_pub_.getNumSubscribers() == 0 ||
				attached_object_pub_.getNumSubscribers() == 0)
		{
			ROS_WARN_STREAM("Waiting for planning scene topics");
			ros::Duration(2.0f).sleep();
		}

		return true;
	}

	bool fetch_parameters()
	{
		ros::NodeHandle nh("~");

		return nh.getParam(ARM_GROUP_PARAM,arm_group_name_) &&
			nh.getParam(TCP_LINK_PARAM,tcp_link_name_) && update_parameters();
	}

	bool update_parameters()
	{
		if(collision_obj_details_.fetch_parameters(COLLISION_OBJECT_PARAM) &&
				attached_obj_details_.fetch_parameters(ATTACHED_OBJECT_PARAM))
		{
			collision_obj_details_.get_collision_object(collision_obj_);
			attached_obj_details_.get_attached_object(attached_obj_);
		}
		else
		{
			return false;
		}

		return true;
	}

	bool process_command(int command)
	{

		// clearing members
		planning_scene_msgs_.world.collision_objects.clear();
		planning_scene_msgs_.robot_state.attached_collision_objects.clear();

		switch(command)
		{
		case command_flags::ADD_COLLISION_OBJECT:

			collision_obj_.operation = moveit_msgs::CollisionObject::ADD;
			collision_object_pub_.publish(collision_obj_);
			break;
		case command_flags::ATTACHED_OBJECT_TO_TCP:

			attached_obj_.object.operation = moveit_msgs::CollisionObject::ADD;
			attached_object_pub_.publish(attached_obj_);
			break;
		case command_flags::UPDATE_COLLISION_OBJECT:

			collision_object_pub_.publish(collision_obj_);
			break;
		case command_flags::REMOVE_COLLISION_OBJECT:

			collision_obj_.operation = moveit_msgs::CollisionObject::REMOVE;
			collision_object_pub_.publish(collision_obj_);
			break;
		case command_flags::DETACH_OBJECT_FROM_TCP:

			attached_obj_.object.operation = moveit_msgs::CollisionObject::REMOVE;
			attached_object_pub_.publish(attached_obj_);
			break;
		case command_flags::ADD_OBJECT_TO_PLANNING_SCENE:

			collision_obj_.operation = moveit_msgs::CollisionObject::REMOVE;// removing first
			planning_scene_msgs_.world.collision_objects.push_back(collision_obj_);
			collision_obj_.operation = moveit_msgs::CollisionObject::ADD;// adding same object with different properties
			planning_scene_msgs_.world.collision_objects.push_back(collision_obj_);
			planning_scene_diff_pub_.publish(planning_scene_msgs_);
			break;
		case command_flags::ATTACH_TO_TCP_IN_PLANNING_SCENE:

			attached_obj_.object.operation = moveit_msgs::CollisionObject::ADD;
			planning_scene_msgs_.robot_state.attached_collision_objects.push_back(attached_obj_);
			planning_scene_diff_pub_.publish(planning_scene_msgs_);
			break;
		case command_flags::UPDATE_OBJECT_IN_PLANNING_SCENE:

			collision_obj_.operation = moveit_msgs::CollisionObject::REMOVE; // removing first
			planning_scene_msgs_.world.collision_objects.push_back(collision_obj_);
			collision_obj_.operation = moveit_msgs::CollisionObject::ADD; // adding same object with different properties
			planning_scene_msgs_.world.collision_objects.push_back(collision_obj_);
			planning_scene_diff_pub_.publish(planning_scene_msgs_);
			break;
		case command_flags::REMOVE_OBJECT_FROM_PLANNING_SCENE:

			collision_obj_.operation = moveit_msgs::CollisionObject::REMOVE;
			planning_scene_msgs_.world.collision_objects.push_back(collision_obj_);
			planning_scene_diff_pub_.publish(planning_scene_msgs_);
			break;
		case command_flags::DETACH_FROM_TCP_IN_PLANNING_SCENE:

			attached_obj_.object.operation = moveit_msgs::CollisionObject::REMOVE;
			planning_scene_msgs_.robot_state.attached_collision_objects.push_back(attached_obj_);
			planning_scene_diff_pub_.publish(planning_scene_msgs_);
			break;

		case command_flags::MOVE_ARM_TO_RANDOM_TARGET:

			move_group_ptr_->setRandomTarget();
			ROS_INFO_STREAM("Moving arm and waiting");
			if(move_group_ptr_->move())
			{
				ROS_INFO_STREAM("Move arm succeeded");
			}
			else
			{
				ROS_ERROR_STREAM("Move arm failed");
			}
			break;

		case command_flags::EXIT:
			return false;

		case command_flags::NONE:
			break;

		default:
			ROS_WARN_STREAM("Invalid entry");
			break;
		}

		return true;
	}

protected:

	// ros publishers
	ros::Publisher planning_scene_diff_pub_;
	ros::Publisher attached_object_pub_;
	ros::Publisher collision_object_pub_;

	// move groups interface
	boost::shared_ptr<move_group_interface::MoveGroup> move_group_ptr_;

	// planning scene
	moveit_msgs::PlanningScene planning_scene_msgs_;

	// messages
	moveit_msgs::AttachedCollisionObject attached_obj_;
	moveit_msgs::CollisionObject collision_obj_;

	// ros parameters
	CollisionObjectDetails collision_obj_details_;
	CollisionObjectDetails attached_obj_details_;
//	std::string world_frame_id_;
	std::string arm_group_name_;
	std::string tcp_link_name_;
//	tf::Vector3 box_position_in_world_;
//	double box_side_length_;

};


int main(int argc,char** argv)
{
	ros::init(argc,argv,"planning_scene_publisher");
	ros::NodeHandle nh;

	PlanningSceneTest test;
	test.run();

	return 0;
}


