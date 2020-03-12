#include <ros/ros.h>
#include <data_handler/hw_node.h>
#include <std_msgs/String.h>

void
callback(const std_msgs::String* msg)
{
	static auto n = 0;
	ROS_INFO("got message [%d]: %s", ++n, msg->data.c_str());
}

int
main(int argc, char** argv)
{

	// initialize node
	ros::init(argc, argv, "dh_listener");
	ros::NodeHandle nh;

	// subscribe
	hw_node::subscribe<std_msgs::String>(&nh, "dh_talker", &callback);

	// main loop
	ros::spin();

	// exit
	return 0;
}