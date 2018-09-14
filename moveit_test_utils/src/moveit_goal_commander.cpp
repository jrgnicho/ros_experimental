#include <ros/ros.h>
#include <iostream>
#include <tuple>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/GetMotionPlan.h>
#include <moveit/robot_state/conversions.h>
#include <eigen_conversions/eigen_msg.h>

static const std::string DEFAULT_PLANNING_GROUP = "manipulator";
static const std::string MOTION_PLAN_SERVICE = "plan_kinematic_path";
static const double DEFAULT_JOINT_TOLERANCE = 1e-3; //radians
static const double DEFAULT_POSITION_TOLERANCE = 1e-4; // 0.1 meters
static const double DEFAULT_ORIENTATION_TOLERANCE = 1e-3; // ~0.1 radians
static const double WAIT_SERVICE_PERIOD = 10.0; // seconds

static const std::string HELP_TEXT = R"(
Enter an array where the first number is the command flag,
This flag is then followed by the values used to create the MoveIt! goal
You can use one of the following numeric commands: 
)";

enum CommandType: int
{
  EXIT = 0,
  JOINT = 1,
  CARTESIAN_POSITION,
  CARTESIAN_ORIENTATION,
  CARTESIAN_POSE,
  SHOW_CMD_SYNTAX
};

static const std::map<int,std::string> CMD_TYPES_MAPPINGS = {{JOINT , "JOINT"}, {CARTESIAN_POSE, "CARTESIAN_POSE"},
                                                 {CARTESIAN_POSITION, "CARTESIAN_POSITION"},
                                                 {CARTESIAN_ORIENTATION, "CARTESIAN_ORIENTATION"},
                                                 {EXIT, "EXIT"},{SHOW_CMD_SYNTAX, "SHOW_CMD_SYNTAX"}};
static const std::map<int,std::string> CMD_HINT_MAPPINGS = {{JOINT , "1 j1 j2 j3 j4 j5 j6 "},
                                                             {CARTESIAN_POSE , "4 px py pz rx ry rz"},
                                                             {CARTESIAN_POSITION , "4 px py pz tol_px tol_py tol_pz"},
                                                             {CARTESIAN_ORIENTATION,"3 rx ry rz tol_rx tol_ry tol_rz"}};

template <class T>
std::string to_string(const std::vector<T>& v)
{
  std::string str = std::accumulate(std::next(v.begin()),v.end(),std::to_string(v.front()),
                                    [](std::string s, T val){
    return s + " " + std::to_string(val);
  });
  return str;
}

std::vector<double> parseInputs(std::string input, std::string delimiter = " ")
{
  std::size_t start_pos = 0;
  std::size_t pos = 0;;
  std::vector<double> vals;

  while(pos != std::string::npos)
  {
    pos = input.find(delimiter,start_pos);
    std::string val_str = input.substr(start_pos, pos - start_pos);
    start_pos = pos + 1;
    double val = boost::lexical_cast<double>(val_str);
    vals.push_back(val);
  }

  return std::move(vals);
}

geometry_msgs::Pose createPose(const std::array<double,6>& pose_data)
{
  using namespace Eigen;
  Eigen::Affine3d pose = Translation3d(pose_data[0],pose_data[1],pose_data[2]) * AngleAxisd(pose_data[3],Vector3d::UnitX()) *
      AngleAxisd(pose_data[4],Vector3d::UnitY()) * AngleAxisd(pose_data[5],Vector3d::UnitZ());

  geometry_msgs::Pose pose_msg;
  tf::poseEigenToMsg(pose,pose_msg);
  return std::move(pose_msg);
}

geometry_msgs::Point createPoint(const std::array<double,3>& point_data)
{
  geometry_msgs::Point point;
  std::tie(point.x,point.y,point.z) = std::make_tuple(point_data[0],point_data[1],point_data[2]);
  return std::move(point);
}

geometry_msgs::Quaternion createQuaternion(const std::array<double,3>& rot_data)
{
  using namespace Eigen;
  geometry_msgs::Quaternion q;
  Quaterniond quat = AngleAxisd(rot_data[0],Vector3d::UnitX()) * AngleAxisd(rot_data[1],Vector3d::UnitY())
      * AngleAxisd(rot_data[2],Vector3d::UnitZ());
  tf::quaternionEigenToMsg(quat,q);
  return std::move(q);
}

moveit::core::RobotStatePtr createRobotState(const std::vector<double>& joint_data, const moveit::core::JointModelGroup* group,
                                                  const moveit::core::RobotModelConstPtr& robot_model)
{
  using namespace moveit::core;
  RobotStatePtr robot_st(new RobotState(robot_model));
  robot_st->setToDefaultValues();
  robot_st->setJointGroupPositions(group,joint_data);
  return std::move(robot_st);
}

int main(int argc,char** argv)
{
  using namespace moveit::core;

  ros::init(argc,argv,"moveit_goal_commander");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(3);
  spinner.start();

  // parameters
  ros::NodeHandle ph("~");
  std::string planning_group = DEFAULT_PLANNING_GROUP;
  ph.param("planning_group",planning_group,planning_group);

  // planning client
  ros::ServiceClient plan_client = nh.serviceClient<moveit_msgs::GetMotionPlan>(MOTION_PLAN_SERVICE);
  if(!plan_client.waitForExistence(ros::Duration(WAIT_SERVICE_PERIOD)))
  {
    ROS_ERROR("Service %s was not found",plan_client.getService().c_str());
    return -1;
  }

  // create move group interface
  moveit::planning_interface::MoveGroupInterface move_group(planning_group);
  move_group.startStateMonitor(5.0);
  moveit::core::RobotModelConstPtr robot_model = move_group.getRobotModel();
  const moveit::core::JointModelGroup* group = robot_model->getJointModelGroup(planning_group);
  std::string tool_link = move_group.getEndEffectorLink();
  std::string reference_frame = move_group.getPlanningFrame();

  std::vector<double> user_input = {1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  CommandType cmd_type_flag = static_cast<CommandType>(user_input.front());

  moveit_msgs::MotionPlanRequest request;
  request.group_name = planning_group;
  request.num_planning_attempts = 4;
  request.max_velocity_scaling_factor = 1.0;
  request.allowed_planning_time = 20.0;
  request.planner_id = "RRTConnectkConfigDefault";
  request.workspace_parameters = moveit_msgs::WorkspaceParameters();

  // creating help strings
  std::string cmd_mappings_str = std::accumulate(CMD_TYPES_MAPPINGS.begin(),CMD_TYPES_MAPPINGS.end(),
                                             std::string(""),[](std::string s, const auto& kv){
    return s + "\t- " + kv.second + ": " + std::to_string(kv.first) + "\n";
  });
  const std::string instructions = HELP_TEXT + cmd_mappings_str;

  const std::string syntax_txt = std::accumulate(CMD_HINT_MAPPINGS.begin(),CMD_HINT_MAPPINGS.end(),
                                           std::string(50,'=') + std::string("\nCommands Syntax:\n"),
                                           [&CMD_TYPES_MAPPINGS](std::string s, const auto& kv){
    std::string name = CMD_TYPES_MAPPINGS.at(kv.first);
    return s + "\t- " + name + ": \n\t\t" + kv.second + "\n";
  }) + std::string(50,'=') + "\n";

  while(ros::ok())
  {

    std::string line;
    std::cout<<instructions;
    std::cout<<std::endl<<"Enter Command: ";
    if(std::getline(std::cin,line))
    {
      ROS_DEBUG("Got line: %s",line.c_str());
    }

    user_input = parseInputs(line);

    std::string vec_str = std::accumulate(std::next(user_input.begin()),user_input.end(),
                                        std::to_string(user_input.front()),[](std::string s, double v){
      return s + " " + std::to_string(v);
    });

    ROS_DEBUG("Parsed Input is: %s", vec_str.c_str());

    if(!user_input.empty())
    {
      cmd_type_flag = static_cast<CommandType>(user_input.front());
    }
    else
    {
      ROS_INFO("No input, exiting");
      return 0;
    }

    // check for non motion commands
    switch(cmd_type_flag)
    {
      case CommandType::EXIT:
        return 0;

      case CommandType::SHOW_CMD_SYNTAX:
        std::cout<<std::endl<<syntax_txt;
        continue;
    }

    ROS_INFO("Requested a '%s' commands",CMD_TYPES_MAPPINGS.at(cmd_type_flag).c_str());

    // capture current robot state
    RobotStatePtr current_state = move_group.getCurrentState();
    if(current_state == nullptr)
    {
      current_state.reset(new RobotState(robot_model));
      current_state->setToDefaultValues();
      ROS_WARN("Failed to capture current robot state, using to default values");
    }
    robot_state::robotStateToRobotStateMsg(*current_state,request.start_state );

    moveit_msgs::Constraints constraint;
    switch( cmd_type_flag)
    {
      case JOINT:
      {
        std::vector<double> joint_data(std::next(user_input.begin()),user_input.end());
        RobotStatePtr goal_state = createRobotState(joint_data,group, robot_model);

        constraint = kinematic_constraints::constructGoalConstraints(
            *goal_state,group,DEFAULT_JOINT_TOLERANCE);
      }
      break;

      case CARTESIAN_POSE:
      {
        geometry_msgs::PoseStamped tool_pose;
        std::array<double,6> pose_data;
        std::copy(std::next(user_input.begin()),user_input.end(),pose_data.begin());
        tool_pose.pose = createPose(pose_data);
        tool_pose.header.frame_id = reference_frame;
        ROS_DEBUG_STREAM("Tool Pose: "<<tool_pose);

        constraint = kinematic_constraints::constructGoalConstraints(
            tool_link, tool_pose, DEFAULT_POSITION_TOLERANCE, DEFAULT_ORIENTATION_TOLERANCE);
      }
      break;

      case CARTESIAN_POSITION:
      {
        geometry_msgs::PoseStamped tool_pose;
        std::array<double,3> point_data;
        std::copy(std::next(user_input.begin()),user_input.begin()+ 4,point_data.begin());
        tool_pose.pose.position = createPoint(point_data);
        tf::quaternionEigenToMsg(Eigen::Quaterniond::Identity(),tool_pose.pose.orientation);
        tool_pose.header.frame_id = reference_frame;
        ROS_DEBUG_STREAM("Tool Pose: "<<tool_pose);

        std::vector<double> pos_tolerances(user_input.begin() + 4,user_input.begin() + 7);
        std::vector<double> orient_tolerances(3,M_PI);
        ROS_DEBUG_STREAM("pos_tolerances: "<<to_string(pos_tolerances));
        ROS_DEBUG_STREAM("orient_tolerances: "<<to_string(orient_tolerances));

        constraint = kinematic_constraints::constructGoalConstraints(
            tool_link, tool_pose, pos_tolerances, orient_tolerances);
        constraint.orientation_constraints.clear();
      }
      break;

      case CARTESIAN_ORIENTATION:
      {
        geometry_msgs::PoseStamped tool_pose;
        std::array<double,3> rot_data;
        std::copy(std::next(user_input.begin()),user_input.begin()+ 4 ,rot_data.begin());
        tool_pose.pose.orientation = createQuaternion(rot_data);
        tool_pose.header.frame_id = reference_frame;
        ROS_DEBUG_STREAM("Tool Pose: "<<tool_pose);

        std::vector<double> pos_tolerances(3,0.0);
        std::vector<double> orient_tolerances(user_input.begin() + 4,user_input.begin() + 7);

        ROS_DEBUG_STREAM("pos_tolerances: "<<to_string(pos_tolerances));
        ROS_DEBUG_STREAM("orient_tolerances: "<<to_string(orient_tolerances));

        constraint = kinematic_constraints::constructGoalConstraints(
            tool_link, tool_pose, pos_tolerances, orient_tolerances);
        constraint.position_constraints.clear();
      }
      break;

      default:

        ROS_ERROR("Unknown command type %i requested", cmd_type_flag);
      continue;
    }

    request.goal_constraints.clear();
    request.goal_constraints.resize(1);
    request.goal_constraints[0] = kinematic_constraints::mergeConstraints(constraint, request.goal_constraints[0]);
    ROS_DEBUG_STREAM("Constructed constraint:\n"<<request.goal_constraints[0]);

    // create motion plan
    moveit_msgs::GetMotionPlan motion_plan;
    motion_plan.request.motion_plan_request = request;
    if(!plan_client.call(motion_plan))
    {
      ROS_ERROR("Motion plan request failed");
      continue;
    }

    const moveit_msgs::MotionPlanResponse& res = motion_plan.response.motion_plan_response;
    if(res.error_code.val != moveit_msgs::MoveItErrorCodes::SUCCESS)
    {
      ROS_ERROR("Planning failed, error code: %i",res.error_code.val);
      continue;
    }

    ROS_INFO("Planning Succeeded");
  }

  return 0;
}
