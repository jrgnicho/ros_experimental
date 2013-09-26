/*
 * grasp_service_server.cpp
 *
 *  Created on: Jun 11, 2013
 *      Author: ros developer 
 */
#include <ros/ros.h>
#include <ros/service_server.h>
#include <pick_and_place/GraspService.h>

const std::string GRASP_SERVICE_NAME = "grasp_service";
class GraspServer
{
public:

	GraspServer()
	{

	}

	~GraspServer()
	{

	}

	void run()
	{
		ros::NodeHandle nh;
		ros::ServiceServer grasp_server = nh.advertiseService(GRASP_SERVICE_NAME,&GraspServer::grasp_callback,this);

		ros::spin();
	}

protected:

	bool grasp_callback(pick_and_place::GraspService::Request& req, pick_and_place::GraspService::Response &res)
	{
		switch(req.grasp_goal.action)
		{
		case pick_and_place::GraspCommand::GRASP:

			ROS_INFO_STREAM("Grasp goal \"GRASP\" accepted");
			break;

		case pick_and_place::GraspCommand::PREGRASP:

			ROS_INFO_STREAM("Grasp goal \"PREGRASP\" accepted");
			break;

		case pick_and_place::GraspCommand::RELEASE:

			ROS_INFO_STREAM("Grasp goal \"RELEASE\" accepted");
			break;

		default:

			ROS_INFO_STREAM("Grasp goal "<<req.grasp_goal.action<< " not supported");
			break;
		}

		return true;
	}

};

int main(int argc, char** argv)
{
	ros::init(argc,argv,"grasp_server");
	GraspServer grasp_server;
	grasp_server.run();

	return 0;
}



