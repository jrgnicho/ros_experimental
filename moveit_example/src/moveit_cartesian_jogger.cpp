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
		static const std::string CURRENT_JOINT_STATES = "joint_states";
		static const std::string NEW_JOINT_STATES = "robot_joint_states";
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
			ros::Duration loop_duration(0.2f);
			while(ros::ok())
			{
				loop_duration.sleep();
				ros::spinOnce();
			}

			return;
		}

	protected:

		bool setup()
		{
			if(fetch_parameters())
			{
				ROS_INFO_STREAM("\nparameters\n"<<"arm group name: "<<arm_group_name_
						<<"\ntcp link name: "<<tcp_link_name_);
			}
			else
			{
				ROS_ERROR_STREAM("parameters not found during setup");
				return false;
			}

			// ros setup
			ros::NodeHandle nh;

			// publisher
			joint_state_pub_ =  nh.advertise<sensor_msgs::JointState>(topics::NEW_JOINT_STATES,1);

			// subscriber
			transform_subs_ = nh.subscribe(topics::TCP_DELTA_TRANSFORM,1,&CartesianJogger::transform_subs_callback,this);

			// moveit setup
			kinematic_model_ptr_ = robot_model_loader::RobotModelLoader(parameters::ROBOT_DESCRIPTION).getModel();
			kinematic_state_ptr_.reset(new robot_state::RobotState(kinematic_model_ptr_));
			ROS_INFO_STREAM("Robot frame: "<<kinematic_model_ptr_->getModelFrame());

			// joint group names
			const std::vector<std::string> &group_names = kinematic_model_ptr_->getJointModelGroupNames();
			std::stringstream ss;
			for(std::vector<std::string>::const_iterator i = group_names.begin(); i < group_names.end();i++)
			{
				ss<<*i<<" ";
			}
			ss<<"]";
			ROS_INFO_STREAM("Group Names: "<<ss.str());

			// joint model group
			const robot_state::JointModelGroup* joint_model_group = kinematic_model_ptr_->getJointModelGroup(arm_group_name_);
			const std::vector<std::string> &joint_names = joint_model_group->getActiveJointModelNames();
			ss.str("[");
			for(std::vector<std::string>::const_iterator i = joint_names.begin(); i < joint_names.end();i++)
			{
				ss<<*i<<" ";
			}
			ss<<"]";
			ROS_INFO_STREAM("Joint Names: "<<ss.str());

			// group info
			joint_model_group->printGroupInfo();

			ROS_INFO_STREAM("Set State from IK allowed: "<<joint_model_group->canSetStateFromIK(tcp_link_name_) ? "Yes": "No");

			return true;

		}

		bool fetch_parameters()
		{
			ros::NodeHandle nh("~");

			return nh.getParam(parameters::ARM_GROUP,arm_group_name_)
					&& nh.getParam(parameters::TCP_LINK,tcp_link_name_);
		}

		void transform_subs_callback(geometry_msgs::TransformStampedConstPtr tf_msg)
		{
			sensor_msgs::JointStateConstPtr current_js_ptr = ros::topic::waitForMessage<sensor_msgs::JointState>(
					topics::CURRENT_JOINT_STATES);

			if(compute_ik(*tf_msg.get(),*current_js_ptr.get(),joint_states_msg_))
			{
				joint_state_pub_.publish(joint_states_msg_);
			}
			else
			{
				ROS_WARN_STREAM("ik solution not found");
			}

		}

		bool compute_ik(const geometry_msgs::TransformStamped &delta_tf_msg,
				const sensor_msgs::JointState& current_js,sensor_msgs::JointState &js)
		{
			ROS_INFO_STREAM("In compute_ik");

			// converting msg
			ROS_INFO_STREAM("converting transform msg");
			tf::Transform delta_tf;
			Eigen::Affine3d delta_tf_aff;
			tf::transformMsgToTF(delta_tf_msg.transform,delta_tf);
			tf::transformTFToEigen(delta_tf,delta_tf_aff);

			ROS_INFO_STREAM("retrieving joint group");
			const moveit::core::JointModelGroup* joint_model_group_ptr = kinematic_model_ptr_->getJointModelGroup(arm_group_name_);

			// set current joint values
			ROS_INFO_STREAM("setting current joint values");
			kinematic_state_ptr_->setToDefaultValues();
			kinematic_state_ptr_->setJointGroupPositions(joint_model_group_ptr,current_js.position);

			// applying transform
			ROS_INFO_STREAM("applying transform");
			const Eigen::Affine3d &tcp_tf = kinematic_state_ptr_->getGlobalLinkTransform(tcp_link_name_);
			Eigen::Affine3d goal_tcp_tf = tcp_tf*delta_tf_aff;

			// print tcp desired pose
			ROS_INFO_STREAM("tcp position: \n"<<goal_tcp_tf.translation());
			ROS_INFO_STREAM("tcp orientation: \n"<<goal_tcp_tf.rotation());

			// solving ik
			ROS_INFO_STREAM("Calling ik");
			bool found = kinematic_state_ptr_->setFromIK(joint_model_group_ptr,
					goal_tcp_tf,tcp_link_name_,
					constants::IK_SOLVER_ATTEMTPTS,
					constants::IK_SOLVER_TIMEOUT);

			// populating joint message
			if(found)
			{
				ROS_INFO_STREAM("construction joint state message");

				const std::vector<std::string> &jn = joint_model_group_ptr->getActiveJointModelNames();
				js.name.assign(jn.begin(),jn.end());
				js.position.clear();
				kinematic_state_ptr_->copyJointGroupPositions(joint_model_group_ptr,js.position);

				js.velocity.assign(js.position.size(),0.0f);
				js.effort.assign(js.position.size(),0.0f);
			}

			ROS_INFO_STREAM("IK solution "<<found ? "found" : "not found");

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
	ros::AsyncSpinner spinner(1);
	spinner.start();

	CartesianJogger c = CartesianJogger();
	c.run();

	spinner.stop();

	return 0;
}



