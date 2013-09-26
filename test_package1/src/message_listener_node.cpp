/*
 * message_listener_node.cpp
 *
 *  Created on: Apr 15, 2013
 *      Author: ros developer 
 */

#include <ros/ros.h>
#include <boost/assign.hpp>
#include <boost/assign/list_of.hpp>
#include <string.h>
#include <basic_package/TestMessage.h>

static const std::string TEST_MESSAGE_TOPIC = "test_topic";

void msg_subscribe_cb(const basic_package::TestMessageConstPtr &msg)
{
	ROS_INFO_STREAM("subscriber callback received message");
}

int main(int argc,char** argv)
{
	ros::init(argc,argv,"message_listener_node");
	ros::NodeHandle nh;

	// setting up subscriber
	ros::Subscriber subscriber = nh.subscribe(TEST_MESSAGE_TOPIC,1,msg_subscribe_cb);

	ros::Duration loop_duration(2.0f);

	while(ros::ok() && loop_duration.sleep())
	{
		ros::spinOnce();
	}

	return 0;
}



