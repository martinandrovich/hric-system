#include <ros/ros.h>
#include <data_handler/hw_node.h>
#include <data_handler/TestMessage.h>

void
test(const data_handler::TestMessage* msg)
{
	std::cout << "test message:\n"
			  << "  > " << "text: " << msg->text << "\n"
			  << "  > " << "n: "    << msg->n << "\n"
			  << "  > " << "f: "    << msg->f << "\n"
			  << "\n";
}

int
main(int argc, char** argv)
{

	// initialize node
	ros::init(argc, argv, "hwlistener");
	ros::NodeHandle nh;

	// subscribe
	// hw_node<data_handler::TestMessage>::subscribe(&nh, "hwtalker", test);
	hw_node::subscribe<data_handler::TestMessage>(&nh, "hwtalker", test);

	// main loop
	ros::spin();

	// exit
	return 0;
}