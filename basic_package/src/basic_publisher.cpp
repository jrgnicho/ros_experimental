/*
 * basic_publisher.cpp
 *
 *  Created on: Apr 12, 2013
 *      Author: ros developer 
 */

#include <ros/ros.h>
#include <basic_package/TestMessage.h>
#include <boost/assign.hpp>
#include <boost/assign/list_of.hpp>

using namespace boost::assign;
int main(int argc,char** argv)
{
	ros::init(argc,argv,"basic_publisher_node");
	ros::NodeHandle nh;

	// publisher setup
	ros::Publisher pub = nh.advertise<basic_package::TestMessage >("test_topic",1);
	basic_package::TestMessage msg;
	msg.float_array = list_of<float>(1.0f)(3.0f)(4.0f)(8.0f);
	msg.int_array = list_of(2)(2)(4)(32);
	msg.string_val = "a string value has been placed in this member of the message template object";

	ros::Duration loop_duration(1.0f);
	ROS_INFO_STREAM("Publishing message");
	while(ros::ok() && loop_duration.sleep())
	{
		pub.publish(msg);
	}

	return 0;
}




