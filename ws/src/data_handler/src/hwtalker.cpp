#include <ros/ros.h>
#include <data_handler/hw_node.h>
#include <data_handler/TestMessage.h>

int
main(int argc, char** argv)
{

	// initialize node
	ros::init(argc, argv, "hwtalker");
	ros::NodeHandle nh;

	// init hw node
	// hw_node<data_handler::TestMessage>::init(nh, 1);
	hw_node::init<data_handler::TestMessage>(nh, 1, "hwtalker");

	// hw_node::publish(data&);
	// hw_node::spin(data&, mutex&);

	// main loop
	ros::Rate r(1);
	while (ros::ok())
	{
		// generate test message
		data_handler::TestMessage msg;
		msg.text = "Hello!";
		msg.n = 420;
		msg.f = 3.1415;

		// publish to data handler
		hw_node::publish(msg);

		// sleep
		r.sleep();
	}

	// exit
	return 0;
}