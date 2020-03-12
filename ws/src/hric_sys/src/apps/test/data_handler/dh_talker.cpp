#include <ros/ros.h>
#include <data_handler/hw_node.h>
#include <std_msgs/String.h>

// #include <data_handler/data_handler.h>

int
main(int argc, char** argv)
{
	// initialize node with handle
	ros::init(argc, argv, "dh_talker");
	ros::NodeHandle nh;

	// initialize hw node and register it with data handler
	// name is optional, default is node name
	hw_node::init<std_msgs::String>(nh, 1, "dh_talker");

	// main loop
	ros::Rate r(1);
	while (ros::ok())
	{
		// generate test message
		static std_msgs::String msg;
		msg.data = "HELLO WORLD!";

		ROS_INFO("Sending msg from dh_talker at 1 Hz.");
		hw_node::publish(msg);

		// sleep
		r.sleep();
	}

	// exit
	return 0;
}