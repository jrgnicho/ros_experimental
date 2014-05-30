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
#include <moveit_msgs/DisplayTrajectory.h>

// boost
#include <boost/assign.hpp>
#include <boost/foreach.hpp>
#include <boost/assign/list_of.hpp>

// topics
static const std::string MOTION_PLAN_PREVIEW = "motion_plan_preview";
static const std::string ATTACHED_COLLISION_OBJECT_TOPIC = "attached_collision_object";
static const std::string COLLISION_OBJECT_TOPIC = "collision_object";
static const std::string PLANNING_SCENE_DIFF_TOPIC = "planning_scene";
static const std::string TCP_POSE_TOPIC_ = "tcp_pose";

// parameters
static std::string COLLISION_OBJECT_PARAM = "collision_object_info";
static std::string TCP_TARGET_PARAM = "tcp_target_info";
static std::string ATTACHED_OBJECT_PARAM = "attached_object_info";
static std::string ARM_GROUP_PARAM = "arm_group";
static std::string TCP_LINK_PARAM = "tcp_link";


using namespace boost::assign;

namespace command_flags
{
	enum CommandFlags
	{
		EXIT = -1,
		ADD_COLLISION_OBJECT = 0,
		REMOVE_COLLISION_OBJECT = 1,
		ATTACH_TO_TCP = 2,
		DETACH_FROM_TCP = 3,
		DETACH_FROM_TCP_AND_REMOVE = 4,
		ADD_OBJECT_TO_PLANNING_SCENE = 5,
		REMOVE_OBJECT_FROM_PLANNING_SCENE = 6,
		ATTACH_TO_TCP_IN_PLANNING_SCENE = 7,
		DETACH_FROM_TCP_IN_PLANNING_SCENE = 8,
		DETACH_FROM_TCP_AND_REMOVE_IN_PLANNING_SCENE = 9,
		MOVE_ARM_TO_RANDOM_TARGET = 10,
		MOVE_ARM_TO_TCP_TARGET = 11,
		MOVE_ARM_TO_TARGET_PREVIEW=12,
		NONE = 14
	};

	static std::map<int,std::string> NAME_MAPPINGS = map_list_of((int)ADD_COLLISION_OBJECT,"ADD COLLISION_OBJECT")
		((int)REMOVE_COLLISION_OBJECT,"REMOVE_COLLISION_OBJECT")
		((int)ATTACH_TO_TCP,"ATTACH_TO_TCP")
		((int)DETACH_FROM_TCP,"DETACH_FROM_TCP")
		((int)DETACH_FROM_TCP_AND_REMOVE,"DETACH_FROM_TCP_AND_REMOVE")
		((int)ADD_OBJECT_TO_PLANNING_SCENE,"ADD_OBJECT_TO_PLANNING_SCENE")
		((int)REMOVE_OBJECT_FROM_PLANNING_SCENE,"REMOVE_OBJECT_FROM_PLANNING_SCENE")
		((int)ATTACH_TO_TCP_IN_PLANNING_SCENE,"ATTACH_TO_TCP_IN_PLANNING_SCENE")
		((int)DETACH_FROM_TCP_IN_PLANNING_SCENE,"DETACH_FROM_TCP_IN_PLANNING_SCENE")
		((int)DETACH_FROM_TCP_AND_REMOVE_IN_PLANNING_SCENE,"DETACH_FROM_TCP_AND_REMOVE_IN_PLANNING_SCENE")
		((int)MOVE_ARM_TO_RANDOM_TARGET,"MOVE_ARM_TO_RANDOM_TARGET")
		((int)MOVE_ARM_TO_TCP_TARGET,"MOVE_ARM_TO_TCP_TARGET")
		((int)MOVE_ARM_TO_TARGET_PREVIEW,"MOVE_ARM_TO_TARGET_PREVIEW")
		((int)EXIT ,"EXIT ");
}

class StructParamLoader
{
public:
	StructParamLoader()
	{

	}

	bool load_parameters(std::string param_name,XmlRpc::XmlRpcValue &p,std::string ns="")
	{
		ros::NodeHandle nh(ns);
		bool success = true;

		// loading structure array
		success = nh.getParam(param_name,p) && p.getType()==p.TypeStruct;

		// verifying all members
		std::list<std::string>::iterator i;
		for(i = field_names_.begin(); i != field_names_.end();i++)
		{
			if(!p.hasMember(*i))
			{
				success = false;
				ROS_ERROR_STREAM("structure field '"<<*i<<"' in parameter "<<nh.getNamespace()+ "/" + param_name <<" not found");
				break;
			}
		}

		return success;

	}

protected:

	std::list<std::string> field_names_;
};

class PoseTargetDetails: public StructParamLoader
{

public:
	PoseTargetDetails()
	{
		// list of fields in array
		field_names_ = list_of("frame_id")
				("x")
				("y")
				("z")
				("rx")
				("ry")
				("rz");
	}

	bool load_parameters(std::string param_name,geometry_msgs::PoseStamped &pose,std::string ns="")
	{
		XmlRpc::XmlRpcValue p;
		tf::Transform t;
		double x,y,z,rx,ry,rz;

		bool success = StructParamLoader::load_parameters(param_name,p,ns);
		if(success)
		{
			x = static_cast<double>(p["x"]);
			y = static_cast<double>(p["y"]);
			z = static_cast<double>(p["z"]);
			rx = static_cast<double>(p["rx"]);
			ry = static_cast<double>(p["ry"]);
			rz = static_cast<double>(p["rz"]);

			t.setOrigin(tf::Vector3(x,y,z));
			t.getBasis().setRPY(rx,ry,rz);
			tf::poseTFToMsg(t,pose.pose);

			pose.header.frame_id = static_cast<std::string>(p["frame_id"]);
		}

		return success;
	}
};

class CollisionObjectDetails: public StructParamLoader
{
public:
	CollisionObjectDetails()
	{

		// list of fields in array
		field_names_ = list_of("id")
				("frame_id")
				("link_name")
				("shape_type")
				("x")
				("y")
				("z")
				("rx")
				("ry")
				("rz")
				("l")
				("w")
				("h");
	}


	bool load_parameters(std::string param_name,moveit_msgs::CollisionObject &cobj,std::string ns="")
	{
		XmlRpc::XmlRpcValue p;
		return load_parameters(param_name,cobj,p,ns);
	}

	bool load_parameters(std::string param_name,moveit_msgs::CollisionObject &cobj,XmlRpc::XmlRpcValue &p,std::string ns="")
	{

		// parsing parameter structure
		bool success = StructParamLoader::load_parameters(param_name,p,ns);
		if(success)
		{
			// creating pose
			tf::Transform t;
			double x,y,z,rx,ry,rz;
			geometry_msgs::Pose pose;

			x = static_cast<double>(p["x"]);
			y = static_cast<double>(p["y"]);
			z = static_cast<double>(p["z"]);
			rx = static_cast<double>(p["rx"]);
			ry = static_cast<double>(p["ry"]);
			rz = static_cast<double>(p["rz"]);

			t.setOrigin(tf::Vector3(x,y,z));
			t.getBasis().setRPY(rx,ry,rz);
			tf::poseTFToMsg(t,pose);

			// creating shape
			shape_msgs::SolidPrimitive shape;
			shape.type = shape_msgs::SolidPrimitive::BOX; // ignoring shape type entry for now
			shape.dimensions.resize(3);
			shape.dimensions[0] = static_cast<double>(p["l"]);
			shape.dimensions[1] = static_cast<double>(p["w"]);
			shape.dimensions[2] = static_cast<double>(p["h"]);

			// filling collision object members
			cobj.primitives.clear();
			cobj.primitive_poses.clear();
			cobj.primitives.push_back(shape);
			cobj.primitive_poses.push_back(pose);
			cobj.header.frame_id = static_cast<std::string>(p["frame_id"]);
			cobj.id = static_cast<std::string>(p["id"]);

		}

		return success;
	}

	bool load_parameters(std::string param_name,moveit_msgs::AttachedCollisionObject& att_obj,std::string ns="")
	{
		XmlRpc::XmlRpcValue p;
		bool success = load_parameters(param_name,att_obj.object,p,ns);
		if(success)
		{
			att_obj.link_name = static_cast<std::string>(p["link_name"]);
		}

		return success;
	}

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
		ros::NodeHandle ph("~");

		if(load_parameters())
		{
			ROS_INFO_STREAM("parameters were read successfully");
		}
		else
		{
			ROS_ERROR_STREAM("failed to read parameters, exiting");
			return false;
		}

		// messages
		collision_obj_.operation = moveit_msgs::CollisionObject::REMOVE;
		attached_obj_.object.operation = moveit_msgs::CollisionObject::REMOVE;

		// ros publishers
		planning_scene_diff_pub_ = nh.advertise<moveit_msgs::PlanningScene>(PLANNING_SCENE_DIFF_TOPIC,1);
		collision_object_pub_ = nh.advertise<moveit_msgs::CollisionObject>(COLLISION_OBJECT_TOPIC,1);
		attached_object_pub_ = nh.advertise<moveit_msgs::AttachedCollisionObject>(ATTACHED_COLLISION_OBJECT_TOPIC,1);
		tcp_pose_pub_ = nh.advertise<geometry_msgs::PoseStamped>(TCP_POSE_TOPIC_,1);
		motion_plan_preview_pub_ = ph.advertise<moveit_msgs::DisplayTrajectory>(MOTION_PLAN_PREVIEW,1);

		// ros timers
		timer_ = nh.createTimer(ros::Duration(0.5f),&PlanningSceneTest::timer_callback,this);

		// setting up planning scene diff message
		planning_scene_msgs_.is_diff = true;

		// setting up move group interface
		move_group_ptr_ = boost::shared_ptr<move_group_interface::MoveGroup>(new move_group_interface::MoveGroup(arm_group_name_));
		move_group_ptr_->setPlanningTime(60.0f);

		// waiting for topics
		while(planning_scene_diff_pub_.getNumSubscribers() == 0 || collision_object_pub_.getNumSubscribers() == 0 ||
				attached_object_pub_.getNumSubscribers() == 0)
		{
			ROS_WARN_STREAM("Waiting for planning scene topics");
			ros::Duration(2.0f).sleep();
		}

		return true;
	}

	void timer_callback(const ros::TimerEvent& e)
	{
		update_parameters();

		// publishing target pose
		tcp_pose_pub_.publish(tcp_pose_);

		// updating planning scene objects
		if(collision_obj_.operation == moveit_msgs::CollisionObject::ADD)
		{
			collision_object_pub_.publish(collision_obj_);
		}

		if(attached_obj_.object.operation == moveit_msgs::CollisionObject::ADD)
		{
			attached_object_pub_.publish(attached_obj_);
		}
	}

	bool load_parameters()
	{
		ros::NodeHandle nh("~");

		return nh.getParam(ARM_GROUP_PARAM,arm_group_name_) &&
			nh.getParam(TCP_LINK_PARAM,tcp_link_name_) && update_parameters();
	}

	bool update_parameters()
	{
		return collision_obj_details_.load_parameters(COLLISION_OBJECT_PARAM,collision_obj_,"~") &&
				collision_obj_details_.load_parameters(ATTACHED_OBJECT_PARAM,attached_obj_,"~") &&
				pose_details_.load_parameters(TCP_TARGET_PARAM,tcp_pose_,"~");
	}

	bool process_command(int command)
	{

		// clearing members
		planning_scene_msgs_.world.collision_objects.clear();
		planning_scene_msgs_.robot_state.attached_collision_objects.clear();

		// plan structure
		move_group_interface::MoveGroup::Plan plan;


		switch(command)
		{
		case command_flags::ADD_COLLISION_OBJECT:

			collision_obj_.operation = moveit_msgs::CollisionObject::ADD;
			collision_object_pub_.publish(collision_obj_);

			break;
		case command_flags::ATTACH_TO_TCP:

			attached_obj_.object.operation = moveit_msgs::CollisionObject::ADD;
			attached_object_pub_.publish(attached_obj_);
			break;
		case command_flags::DETACH_FROM_TCP_AND_REMOVE:

			attached_obj_.object.operation = moveit_msgs::CollisionObject::REMOVE;
			attached_object_pub_.publish(attached_obj_);
			ros::Duration(0.5f).sleep();
			collision_object_pub_.publish(attached_obj_.object);
			break;
		case command_flags::REMOVE_COLLISION_OBJECT:

			collision_obj_.operation = moveit_msgs::CollisionObject::REMOVE;
			collision_object_pub_.publish(collision_obj_);
			break;
		case command_flags::DETACH_FROM_TCP:

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
		case command_flags::DETACH_FROM_TCP_AND_REMOVE_IN_PLANNING_SCENE:

			attached_obj_.object.operation = moveit_msgs::CollisionObject::REMOVE;
			planning_scene_msgs_.robot_state.attached_collision_objects.push_back(attached_obj_);
			planning_scene_msgs_.world.collision_objects.push_back(attached_obj_.object);
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

		case command_flags::MOVE_ARM_TO_TCP_TARGET:

			move_group_ptr_->setEndEffectorLink(tcp_link_name_);
			move_group_ptr_->clearPoseTargets();
			if(move_group_ptr_->setPoseTarget(tcp_pose_,tcp_link_name_) && move_group_ptr_->move())
			{
				ROS_INFO_STREAM("Moved tcp to target :\n"<<tcp_pose_);
			}
			else
			{
				ROS_ERROR_STREAM("Moved tcp to target error :\n"<<tcp_pose_);
			}

			break;

		case command_flags::MOVE_ARM_TO_TARGET_PREVIEW:

			move_group_ptr_->setEndEffectorLink(tcp_link_name_);
			move_group_ptr_->clearPoseTargets();
			if(move_group_ptr_->setPoseTarget(tcp_pose_,tcp_link_name_) && move_group_ptr_->plan(plan))
			{

				display_traj_.trajectory_start = plan.start_state_;
				display_traj_.trajectory.clear();
				display_traj_.trajectory.push_back( plan.trajectory_);
				display_traj_.model_id = move_group_ptr_->getName();
				motion_plan_preview_pub_.publish(display_traj_);
				ROS_INFO_STREAM("Plan to tcp to target succeeded:\n"<<tcp_pose_);
			}
			else
			{
				ROS_ERROR_STREAM("Plan to tcp to target error :\n"<<tcp_pose_);
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
	ros::Publisher tcp_pose_pub_;
	ros::Publisher motion_plan_preview_pub_;

	ros::Timer timer_;

	// move groups interface
	boost::shared_ptr<move_group_interface::MoveGroup> move_group_ptr_;

	// planning scene
	moveit_msgs::PlanningScene planning_scene_msgs_;

	// messages
	moveit_msgs::AttachedCollisionObject attached_obj_;
	moveit_msgs::CollisionObject collision_obj_;
	moveit_msgs::DisplayTrajectory display_traj_;

	// ros parameters
	geometry_msgs::PoseStamped tcp_pose_;
	PoseTargetDetails pose_details_;
	CollisionObjectDetails collision_obj_details_;
	std::string arm_group_name_;
	std::string tcp_link_name_;


};


int main(int argc,char** argv)
{
	ros::init(argc,argv,"planning_scene_publisher");
	ros::NodeHandle nh;

	PlanningSceneTest test;
	test.run();

	return 0;
}


