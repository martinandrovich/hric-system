#include <atomic>
#include <mutex>
#include <string>

#include <ros/ros.h>
#include <controller_manager/controller_manager.h>

#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64MultiArray.h>

#include <franka_gazebo/dynamics.h>

namespace sim_control
{

	const std::string CONTROLLER_NAME = "panda_joint_effort_controller";

	bool
	start_controller()
	{
		ros::service::waitForService("controller_manager/switch_controller");
		ros::Duration(2).sleep();

		ROS_INFO("Switching to '%s' controller.", CONTROLLER_NAME.c_str());

		controller_manager_msgs::SwitchController srv_switch_controller;
		srv_switch_controller.request.start_controllers = { CONTROLLER_NAME };

		return (ros::service::call("controller_manager/switch_controller", srv_switch_controller) and srv_switch_controller.response.ok);
	}
}

int
main(int argc, char** argv)
{

	// initialize node
	ros::init(argc, argv, "gravity_control");
	ros::NodeHandle nh;

	// start 'panda_joint_trajectory_controller' when controller_manager is loaded
	sim_control::start_controller();

	// subsribers
	auto sub_joint_state = nh.subscribe<sensor_msgs::JointState>("/joint_states", 100, franka_gazebo::dynamics::callback_joint_state);

	// publishers
	auto pub_tau_d = nh.advertise<std_msgs::Float64MultiArray>(sim_control::CONTROLLER_NAME + "/command", 100);

	// initialize franka panda dynamics library
	franka_gazebo::dynamics::init();

	// start async callbacks
	auto spinner = ros::AsyncSpinner(0);
	spinner.start();

	// desired torques
	std_msgs::Float64MultiArray tau_d;
	tau_d.data.resize(franka_gazebo::dynamics::NUM_JOINTS);

	// log info
	ROS_INFO("[OK] Initialized '%s' node.", ros::this_node::getName().c_str());

	std::cin.get();

	// main loop
	ros::Rate rate(1); // hz
	while (ros::ok)
	{
		using namespace sim_control;
		using namespace franka_gazebo;

		// read and output actual + desired joint states
		ROS_WARN("Gravity controller:");
		
		if (auto G = dynamics::get_gravity())
		{
			for (size_t i = 0; i < G->data.size(); ++i)
			{
				tau_d.data[i] = G->data[i] * 10.0;
				ROS_INFO("tau_d[%lu]: %f", i, tau_d.data[i]);
			}
		}

		// publish desired torques
		pub_tau_d.publish(tau_d);

		// sleep for remainding time
		rate.sleep();
	}

	// exit
	ros::waitForShutdown();
	return 0;
}