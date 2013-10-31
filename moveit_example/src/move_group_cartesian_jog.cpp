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
#include <tf2/transform_datatypes.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

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
		static const std::string WORLD_FRAME_ID = "world_frame";
		static const double LOOP_CYCLE_TIME = 0.1f; // seconds
		static const double TIME_TOLERANCE = 0.001f; // seconds
		static const double ANGLE_TOLERANCE= 0.01f; // radians
		static const double DIFF_TOLERANCE = 0.001;
		static const double PLANNING_TIME = 4.0f; // seconds
		static const int IK_SOLVER_ATTEMTPTS = 10;
		static const double IK_SOLVER_TIMEOUT = 0.1f;
	}

	class MoveCartesianJog
	{
		typedef boost::shared_ptr<move_group_interface::MoveGroup> MoveGroupPtr;

	public:
		MoveCartesianJog():
			last_incremental_tf_(tf2::Transform::getIdentity()),
			total_traj_time_elapsed_(0.0f)
		{

		}

		~MoveCartesianJog()
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
			ros::Duration loop_duration(constants::LOOP_CYCLE_TIME);
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

			// moveit setup
			move_group_ptr_.reset(new move_group_interface::MoveGroup(parameters::ARM_GROUP));
			move_group_ptr_->setEndEffectorLink(parameters::TCP_LINK);
			move_group_ptr_->setPoseReferenceFrame(constants::WORLD_FRAME_ID);
			move_group_ptr_->setPlanningTime(constants::PLANNING_TIME);


			// subscriber
			transform_subs_ = nh.subscribe(topics::TCP_DELTA_TRANSFORM,1,&MoveCartesianJog::transform_subs_callback,this);

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
			move_to_target(*tf_msg.get());
		}

		void move_to_target(const geometry_msgs::TransformStamped &dt_msg)
		{
			// current pose
			geometry_msgs::PoseStamped world_to_tcp_pose = move_group_ptr_->getCurrentPose(tcp_link_name_);

			// converting pose to tf
			tf::Transform world_to_tcp_tf;
			tf::poseMsgToTF(world_to_tcp_pose.pose,world_to_tcp_tf);

			// applying transform
			tf::Transform dt_tf;
			tf::transformMsgToTF(dt_msg.transform,dt_tf);
			world_to_tcp_tf = world_to_tcp_tf * dt_tf;

			// set new pose
			tf::poseTFToMsg(world_to_tcp_tf,world_to_tcp_pose.pose);
			move_group_ptr_->setPoseTarget(world_to_tcp_pose,tcp_link_name_);

			// planning
			move_group_interface::MoveGroup::Plan path_plan;
			move_group_ptr_->plan(path_plan);

			set_joint_trajectory(path_plan.trajectory_.joint_trajectory.points);


		}

		bool compute_new_trajectory(const geometry_msgs::TransformStamped dt_msg)
		{
			tf2::Transform dt_tf;
			const geometry_msgs::Vector3 &p = dt_msg.transform.translation;
			const geometry_msgs::Quaternion &q = dt_msg.transform.rotation;
			dt_tf.setOrigin(tf2::Vector3(p.x,p.y,p.z));
			dt_tf.setRotation(tf2::Quaternion(q.x,q.y,q.z,q.w));

			// check for equality
			//if(equal())



			return false;
		}

		bool equal(const tf2::Transform &t1,const tf2::Transform &t2,double tolerance)
		{

			const tf2::Matrix3x3 &m1 = t1.getBasis();
			const tf2::Matrix3x3 &m2 = t2.getBasis();
			const tf2::Vector3 &p1 = t1.getOrigin();
			const tf2::Vector3 &p2 = t2.getOrigin();


			// comparing bases
			bool equal = true;
			for(int i = 0;(i < 3 && equal);i++)
			{
				if(std::abs(p1.m_floats[i] - p2.m_floats[i]) > tolerance)
				{
					equal = false;
					break;
				}

				for(int j = 0;j < 3 ;j++)
				{
					if(std::abs(m1.getRow(i).m_floats[j] - m2.getRow(i).m_floats[j]) > tolerance)
					{
						equal = false;
						break;
					}
				}
			}

			return equal;
		}

		void set_joint_trajectory(const std::vector<trajectory_msgs::JointTrajectoryPoint> &joint_trajectory)
		{
			total_traj_time_elapsed_ = 0;
			joint_trajectory_.assign(joint_trajectory.begin(),joint_trajectory.end());

		}

		bool get_next_joint_positions(std::vector<double> &joint_positions, double time_from_traj_start)
		{
			bool position_available = true;

			while(joint_trajectory_.size()>0)
			{
				trajectory_msgs::JointTrajectoryPoint &sp = *joint_trajectory_.begin();
				if(std::abs(sp.time_from_start.toSec() - time_from_traj_start) >= constants::TIME_TOLERANCE)
				{
					joint_positions.assign(sp.positions.begin(),sp.positions.end());
					joint_trajectory_.pop_front();
					break;
				}
				else
				{
					joint_trajectory_.pop_front();
				}
			}

			if(position_available)
			{
				// check time elapsed

				trajectory_msgs::JointTrajectoryPoint &sp = *joint_trajectory_.begin();
				joint_positions.assign(sp.positions.begin(),sp.positions.end());
				joint_trajectory_.pop_front();
			}
			return position_available;
		}

		void publish_joint_state()
		{
			static std::vector<double> joints;
			total_traj_time_elapsed_ = total_traj_time_elapsed_ + constants::LOOP_CYCLE_TIME;
			if(get_next_joint_positions(joints,total_traj_time_elapsed_))
			{

			}
		}


	protected:

		// parameters
		std::string arm_group_name_;
		std::string tcp_link_name_;

		// moveit
		MoveGroupPtr move_group_ptr_;

		// subscriber
		ros::Subscriber transform_subs_;

		// motion control
		std::list<trajectory_msgs::JointTrajectoryPoint> joint_trajectory_;
		tf2::Transform last_incremental_tf_;
		double total_traj_time_elapsed_;

	};

}

int main(int argc,char** argv)
{
	using namespace cartesian_jogger;
	ros::init(argc,argv,"move_group_cartesian_jog");
	ros::AsyncSpinner spinner(1);
	spinner.start();

	MoveCartesianJog c = MoveCartesianJog();
	c.run();

	spinner.stop();

	return 0;
}



