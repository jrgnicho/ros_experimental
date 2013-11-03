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
		static const double TIME_TOLERANCE = 0.001f; // second
		static const double MIN_ALLOWED_TCP_STEP = 0.01f; //meters
		static const double MAX_TCP_JUMP_THRESHOLD = 0.01f; // meters
		static const double MATRIX_DIFF_TOLERANCE = 0.001;
		static const double PLANNING_TIME = 4.0f; // seconds
		static const int NUM_CARTESIAN_POINTS = 40;
	}

	class MoveCartesianJog
	{
		typedef boost::shared_ptr<move_group_interface::MoveGroup> MoveGroupPtr;

	public:
		MoveCartesianJog():
			differential_tf_(tf::Transform::getIdentity()),
			new_differential_tf_(tf::Transform::getIdentity()),
			motion_halted_(true)
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
				publish_joint_state();
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

			// messages
			joint_states_msg_.name.assign(move_group_ptr_->getJoints().begin(),
					move_group_ptr_->getJoints().end());
			joint_states_msg_.position.clear();
			joint_states_msg_.velocity.assign(joint_states_msg_.name.size(),0.0f);
			joint_states_msg_.effort.assign(joint_states_msg_.name.size(),0.0f);

			// publisher
			joint_state_pub_ =  nh.advertise<sensor_msgs::JointState>(topics::NEW_JOINT_STATES,1);

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
			// saving new differential tf
			tf::transformMsgToTF(tf_msg->transform,new_differential_tf_);
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

			//set_joint_trajectory(path_plan.trajectory_.joint_trajectory.points);
			joint_trajectory_.assign(path_plan.trajectory_.joint_trajectory.points.begin(),
					path_plan.trajectory_.joint_trajectory.points.end());

		}

		// Computes new trajectory from differenctial transform 'diff_tf' in units of m/s and rad/s
		bool compute_new_trajectory(const tf::Transform &diff_tf,
				std::list<trajectory_msgs::JointTrajectoryPoint> &joint_trajectory)
		{
			tf::Transform incr_tf; // incremental transform in units of meters & radians

			// setting incremental transform using loop cycle time
			tf::Vector3 delta_rpy;
			const tf::Matrix3x3 &dm = diff_tf.getBasis();
			dm.getRPY(delta_rpy.m_floats[0],delta_rpy.m_floats[1],delta_rpy.m_floats[2]);
			delta_rpy*=constants::LOOP_CYCLE_TIME; // multiplying velocity by loop cycle time.
			incr_tf.setOrigin(diff_tf.getOrigin() * constants::LOOP_CYCLE_TIME);
			incr_tf.getBasis().setRPY(delta_rpy.getX(),delta_rpy.getY(),delta_rpy.getZ());

			// getting current transform
			tf::Transform tcp_tf = tf::Transform();
			tf::poseMsgToTF(move_group_ptr_->getCurrentPose(tcp_link_name_).pose,tcp_tf);

			// creating trajectory
			std::vector<geometry_msgs::Pose> way_points;
			way_points.resize(constants::NUM_CARTESIAN_POINTS);
			tf::Transform t = tcp_tf;
			for(int i = 0;i < constants::NUM_CARTESIAN_POINTS; i++)
			{
				t*=incr_tf;
				tf::poseTFToMsg(t,way_points[i]);
			}

			// planning trajectory
			double tcp_step = incr_tf.getOrigin().length();
			moveit_msgs::RobotTrajectory traj;
			bool result = (move_group_ptr_->computeCartesianPath(way_points,
					tcp_step > constants::MIN_ALLOWED_TCP_STEP ? tcp_step : constants::MIN_ALLOWED_TCP_STEP,
							constants::MAX_TCP_JUMP_THRESHOLD,traj,true) > 0);
			if(result)
			{
				joint_trajectory.assign(traj.joint_trajectory.points.begin(),traj.joint_trajectory.points.end());
			}

			return result;
		}

		bool equal(const tf::Transform &t1,const tf::Transform &t2,double tolerance)
		{

			const tf::Matrix3x3 &m1 = t1.getBasis();
			const tf::Matrix3x3 &m2 = t2.getBasis();
			const tf::Vector3 &p1 = t1.getOrigin();
			const tf::Vector3 &p2 = t2.getOrigin();


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

		bool get_next_joint_positions(std::vector<double> &joint_positions)
		{
			bool position_available = joint_trajectory_.size()>0;

			if(position_available)
			{
				trajectory_msgs::JointTrajectoryPoint &p = *joint_trajectory_.begin();
				joint_positions.assign(p.positions.begin(),p.positions.end());
				joint_trajectory_.pop_front(); // removing point
			}

			return position_available;
		}

		void update_trajectory()
		{
			// check if new tf is not identity or same as last differential received
			motion_halted_ = equal(new_differential_tf_,tf::Transform::getIdentity(),constants::MATRIX_DIFF_TOLERANCE);
			if(!motion_halted_)
			{
				// compute when new differential transform is received
				if(!equal(differential_tf_,new_differential_tf_,constants::MATRIX_DIFF_TOLERANCE))
				{
					differential_tf_ = new_differential_tf_;
					compute_new_trajectory(differential_tf_,joint_trajectory_);
				}
				else
				{
					// compute when no points remain
					if(joint_trajectory_.size()==0)
					{
						compute_new_trajectory(differential_tf_,joint_trajectory_);
					}
				}
			}
		}

		void publish_joint_state()
		{
			joint_states_msg_.position.clear();
			update_trajectory();
			if(!motion_halted_ && get_next_joint_positions(joint_states_msg_.position))
			{
				joint_state_pub_.publish(joint_states_msg_);
			}
		}


	protected:

		// parameters
		std::string arm_group_name_;
		std::string tcp_link_name_;

		// moveit
		MoveGroupPtr move_group_ptr_;

		// publisher
		ros::Publisher joint_state_pub_;

		// subscriber
		ros::Subscriber transform_subs_;

		// messages
		sensor_msgs::JointState joint_states_msg_;

		// motion control
		std::list<trajectory_msgs::JointTrajectoryPoint> joint_trajectory_;
		tf::Transform differential_tf_;
		tf::Transform new_differential_tf_;
		bool motion_halted_;

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



