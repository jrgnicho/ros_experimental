/*
 * pick_and_place_node.cpp
 *
 *  Created on: Jun 10, 2013
 *      Author: ros developer 
 */


#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <moveit/move_group_interface/move_group.h>
#include <geometry_msgs/Pose.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <pick_and_place/GraspService.h>

// aliases
typedef boost::shared_ptr<move_group_interface::MoveGroup> MoveGroupPtr;

class RobotPickPlace
{
public:

	RobotPickPlace():
		grasp_service_name_("grasp_service"),
		arm_group_name_("manipulator"),
		tcp_link_name("tcp_frame"),
		wrist_link_name_("ee_link"),
		world_frame_id_("world_frame"),
		pick_target_frame_id_("ar_tag"),
		home_pose_name_("home"),
		wait_pose_name_("wait"),
		world_to_place_tf_(tf::Transform::getIdentity()),
		retreat_distance_(0.05f),
		approach_distance_(0.05f)
	{

	}

	~RobotPickPlace()
	{

	}

	void run()
	{
		ros::NodeHandle nh;
		tf::StampedTransform world_to_box_pick_tf;
		tf::Transform world_to_wrist_tf;
		std::vector<geometry_msgs::Pose> pick_poses, place_poses;

		if(setup())
		{
			ROS_INFO_STREAM("Setup completed");
		}
		else
		{
			ROS_ERROR_STREAM("Setup error, exiting");
			return;
		}

		while(ros::ok())
		{
			// resetting arrays
			pick_poses.clear();
			place_poses.clear();

			// moving home
			move_group_ptr_->setNamedTarget(home_pose_name_);
			ROS_INFO_STREAM("Moving arm to "<<home_pose_name_<<" pose");
			if(!move_group_ptr_->move()){ ROS_ERROR_STREAM("Move error");break; }

			// moving to wait
			move_group_ptr_->setNamedTarget(wait_pose_name_);
			ROS_INFO_STREAM("Moving arm to "<<wait_pose_name_<<" pose");
			if(!move_group_ptr_->move()){ ROS_ERROR_STREAM("Move error");break; }

			// opening gripper
			if(!set_grasp(pick_and_place::GraspCommand::RELEASE)){ break; }

			// detecting box pick point
			tf_listener_.lookupTransform(world_frame_id_,pick_target_frame_id_,ros::Time(0.0f),world_to_box_pick_tf);

			// finding wrist pose at pick point
			create_wrist_pose_at_target(tcp_link_name,wrist_link_name_,world_to_box_pick_tf,world_to_wrist_tf);

			// creating pick poses for the wrist
			create_manipulation_poses(approach_distance_,retreat_distance_,world_to_wrist_tf,pick_poses);

			// finding wrist pose at place
			create_wrist_pose_at_target(tcp_link_name,wrist_link_name_,world_to_place_tf_,world_to_wrist_tf);

			// creating place poses for the wrist
			create_manipulation_poses(approach_distance_,retreat_distance_,world_to_wrist_tf,place_poses);

			// moving through pick poses
			ROS_INFO_STREAM("Moving through pick poses");
			if(move_through_poses(pick_poses,wrist_link_name_,pick_and_place::GraspCommand::GRASP,0))
			{
				ROS_INFO_STREAM("Pick moves succeeded");
			}
			else
			{
				ROS_ERROR_STREAM("Pick moves error, exiting");
				break;
			}

			// moving through pick poses
			ROS_INFO_STREAM("Moving through place poses");
			if(move_through_poses(place_poses,wrist_link_name_,pick_and_place::GraspCommand::RELEASE,1))
			{
				ROS_INFO_STREAM("Place moves succeeded");
			}
			else
			{
				ROS_ERROR_STREAM("Place moves error, exiting");
				break;
			}

			// moving home
			move_group_ptr_->setNamedTarget(home_pose_name_);
			ROS_INFO_STREAM("Moving arm to "<<home_pose_name_<<" pose");
			if(!move_group_ptr_->move()){ ROS_ERROR_STREAM("Move error");break; }

		}


	}

protected:

	bool fetch_parameters()
	{
		ros::NodeHandle nh("~");
		double x, y, z;

		if(nh.getParam("arm_group_name",arm_group_name_)
		  && nh.getParam("tcp_link_name",tcp_link_name)
		  && nh.getParam("wrist_link_name",wrist_link_name_)
		  && nh.getParam("world_frame_id",world_frame_id_)
		  && nh.getParam("home_pose_name",home_pose_name_)
		  && nh.getParam("wait_pose_name",wait_pose_name_)
		  && nh.getParam("tag_frame_id",pick_target_frame_id_)
		  && nh.getParam("box_place_x",x)
		  && nh.getParam("box_place_y",y)
		  && nh.getParam("box_place_z",z)
		  && nh.getParam("retreat_distance",retreat_distance_)
		  && nh.getParam("approach_distance",approach_distance_))
		{
			world_to_place_tf_.setOrigin(tf::Vector3(x,y,z));
			return true;
		}
		else
		{
			return false;
		}
	}

	bool setup()
	{
		ros::NodeHandle nh;

		if(fetch_parameters())
		{
			ROS_INFO_STREAM("Parameters found");
		}
		else
		{
			ROS_ERROR_STREAM("Parameters not found");
			return false;
		}

		// moveit initialization
		move_group_ptr_ = MoveGroupPtr(new move_group_interface::MoveGroup(arm_group_name_));
		move_group_ptr_->setPoseReferenceFrame(world_frame_id_); // plan relative to the world frame

		// grasp service client
		grasp_client_ = nh.serviceClient<pick_and_place::GraspService>(grasp_service_name_,true);

		// waiting for client
		if(grasp_client_.waitForExistence(ros::Duration(5.0f)))
		{
			ROS_INFO_STREAM("Grasp service found");
		}
		else
		{
			ROS_ERROR_STREAM("Grasp service timeout");
			return false;
		}

		return true;
	}

	void create_manipulation_poses(double approach, double retreat,const tf::Transform& target_tf,
			std::vector<geometry_msgs::Pose>& poses)
	{
		using namespace tf;

		geometry_msgs::Pose start_pose, target_pose, end_pose;

		// creating start pose by applying a translation along +z by approach distance
		tf::poseTFToMsg(Transform(Quaternion::getIdentity(),Vector3(0,0,approach))*target_tf,start_pose);

		// converting target pose
		tf::poseTFToMsg(target_tf,target_pose);

		// creating end pose by applying a translation along +z by retreat distance
		tf::poseTFToMsg(Transform(Quaternion::getIdentity(),Vector3(0,0,retreat))*target_tf,end_pose);

		poses.clear();
		poses.push_back(start_pose);
		poses.push_back(target_pose);
		poses.push_back(end_pose);
	}

	void create_wrist_pose_at_target(std::string tcp_link, std::string wrist_link,const tf::Transform& world_to_target_tf,
			tf::Transform& world_to_wrist_tf)
	{
		tf::StampedTransform tcp_to_wrist_tf;
		tf::Transform world_to_tcp_tf;

		// solving tcp pose at target
		world_to_tcp_tf.setOrigin(world_to_target_tf.getOrigin());
		world_to_tcp_tf.setRotation(tf::Quaternion(M_PI,0,M_PI_2));

		// finding transform from tcp to wrist
		tf_listener_.lookupTransform(tcp_link,wrist_link,ros::Time(0.0f),tcp_to_wrist_tf);

		// applying transform to find wrist pose
		world_to_wrist_tf = world_to_tcp_tf * tcp_to_wrist_tf;
	}

	bool move_through_poses(const std::vector<geometry_msgs::Pose>& poses,std::string ee_link,
			int grasp_command_flag, int grasp_on_index)
	{

		for(std::size_t i = 0; i < poses.size(); i++)
		{
			move_group_ptr_->setPoseTarget(poses[i],ee_link);
			if(move_group_ptr_->move())
			{
				ROS_INFO_STREAM("Move "<<i<<" succeeded");

				if(grasp_on_index == i)
				{
					if(!set_grasp(grasp_command_flag)){ break;}
				}
			}
			else
			{
				ROS_ERROR_STREAM("Move "<< i << "error ");
				return false;
			}
		}

		return true;
	}

	bool set_grasp(int grasp_command_flag)
	{
		pick_and_place::GraspService grasp_command;
		grasp_command.request.grasp_goal.action = grasp_command_flag;

		if(grasp_client_.call(grasp_command))
		{
			ROS_INFO_STREAM("Gripper set");
		}
		else
		{
			ROS_ERROR_STREAM("Gripper service error");
			return false;
		}

		return true;
	}

protected:

	// ros setup
	ros::NodeHandle nh;

	// ros service
	ros::ServiceClient grasp_client_;

	// ros topics
	std::string grasp_service_name_;  // Service name used to control suction gripper

	// transform listener
	tf::TransformListener tf_listener_;

	// moveit interface
	MoveGroupPtr move_group_ptr_;

	// parameters
	std::string arm_group_name_;  // MoveIt Planning Group associated with the robot arm
	std::string tcp_link_name;    // Link / frame name for the suction gripper tool-tip
	std::string wrist_link_name_; // Link / frame name for the robot wrist tool-flange
	std::string world_frame_id_;  // Frame name for the fixed world reference frame
	std::string pick_target_frame_id_; // Frame name for pick point
	std::string home_pose_name_;  // Named pose for robot Home position (set in SRDF)
	std::string wait_pose_name_;  // Named pose for robot WAIT position (set in SRDF)
	tf::Transform world_to_place_tf_;  // Transform from the WORLD frame to the PLACE location
	double retreat_distance_;     // Distance to back away from pick/place pose after grasp/release
	double approach_distance_;    // Distance to stand off from pick/place pose before grasp/release

};

int main(int argc,char** argv)
{
	ros::init(argc,argv,"pick_and_place_node");
	ros::NodeHandle nh;
	ros::AsyncSpinner spinner(2);
	spinner.start();

	RobotPickPlace robot_navigator;
	robot_navigator.run();

	return 0;

}
