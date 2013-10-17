/*
 * moveit_cartesian_jogger.cpp
 *
 *  Created on: Oct 17, 2013
 *      Author: ros developer 
 */

#include <string.h>
#include <vector>

#include <ros/ros.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Pose.h>
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>

// moveit
#include <moveit/move_group_interface/move_group.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
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

namespace cartesian_jogger
{

	namespace parameters
	{
		static const std::string ARM_GROUP= "arm_group";
		static const std::string TCP_LINK= "tcp_link";
		static const std::string ROBOT_DESCRIPTION = "robot_description";
	}

	namespace topics
	{
		static const std::string JOINT_STATES = "robot_joint_states";
		static const std::string TCP_DELTA_TRANSFORM = "tcp_delta_transform";
	}

	namespace constants
	{
		static const int IK_SOLVER_ATTEMTPTS = 10;
		static const double IK_SOLVER_TIMEOUT = 0.1f;
	}

	class CartesianJogger
	{

	public:
		CartesianJogger()
		{

		}

		~CartesianJogger()
		{

		}

		void run()
		{
			ros::NodeHandle nh;

			// initialize
			if(!setup())
			{
				ROS_ERROR_STREAM("Setup failed");
				return;
			}

			ROS_INFO_STREAM("Started node");
			ros::Duration loop_duration(0.2);
			while(ros::ok())
			{
				loop_duration.sleep();
			}

			return;
		}

	protected:

		bool setup()
		{
			// ros setup
			ros::NodeHandle nh("~");



			// publisher
			joint_state_pub_ =  nh.advertise<sensor_msgs::JointState>(topics::JOINT_STATES,1);

			// subscriber
			transform_subs_ = nh.subscribe(topics::TCP_DELTA_TRANSFORM,1,&CartesianJogger::transform_subs_callback,this);

			// moveit setup
			kinematic_model_ptr_ = robot_model_loader::RobotModelLoader(parameters::ROBOT_DESCRIPTION).getModel();
			kinematic_state_ptr_ = moveit::core::RobotStatePtr(new robot_state::RobotState(kinematic_model_ptr_));

			return fetch_parameters();
		}

		bool fetch_parameters()
		{
			ros::NodeHandle nh("~");

			return nh.getParam(parameters::ARM_GROUP,arm_group_name_)
					&& nh.getParam(parameters::TCP_LINK,tcp_link_name_);
		}

		void transform_subs_callback(geometry_msgs::TransformStampedConstPtr tf_msg)
		{
			if(compute_ik(*tf_msg.get(),joint_states_msg_))
			{
				joint_state_pub_.publish(joint_states_msg_);
			}

		}

		bool compute_ik(const geometry_msgs::TransformStamped &delta_tf_msg,sensor_msgs::JointState &js)
		{
			// converting msg
			tf::Transform delta_tf;
			Eigen::Affine3d delta_tf_aff;
			tf::transformMsgToTF(delta_tf_msg.transform,delta_tf);
			tf::transformTFToEigen(delta_tf,delta_tf_aff);

			// applying transform
			const Eigen::Affine3d &tcp_tf = kinematic_state_ptr_->getGlobalLinkTransform(tcp_link_name_);
			Eigen::Affine3d goal_tcp_tf = tcp_tf*delta_tf_aff;

			// solving ik
			const moveit::core::JointModelGroup* joint_model_group_ptr = kinematic_model_ptr_->getJointModelGroup(arm_group_name_);
			bool found = kinematic_state_ptr_->setFromIK(joint_model_group_ptr,
					goal_tcp_tf,
					constants::IK_SOLVER_ATTEMTPTS,
					constants::IK_SOLVER_TIMEOUT);

			// populating joint message
			if(found)
			{
				const std::vector<std::string> &jn = joint_model_group_ptr->getActiveJointModelNames();
				js.name.assign(jn.begin(),jn.end());
				js.position.clear();
				kinematic_state_ptr_->copyJointGroupPositions(joint_model_group_ptr,js.position);

				js.velocity.assign(js.position.size(),0.0f);
				js.effort.assign(js.position.size(),0.0f);
			}

			return found;
		}

	protected:

		// parameters
		std::string arm_group_name_;
		std::string tcp_link_name_;

		// moveit
		moveit::core::RobotModelPtr kinematic_model_ptr_;
		moveit::core::RobotStatePtr kinematic_state_ptr_;

		// publisher
		ros::Publisher joint_state_pub_;

		// subscriber
		ros::Subscriber transform_subs_;

		// messages
		sensor_msgs::JointState joint_states_msg_;
	};

}

int main(int argc,char** argv)
{
	using namespace cartesian_jogger;
	ros::init(argc,argv,"moveit_cartesian_jogger");
	CartesianJogger c = CartesianJogger();
	c.run();

	return 0;
}



