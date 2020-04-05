#include <atomic>
#include <mutex>
#include <string>

#include <ros/ros.h>
#include <controller_manager/controller_manager.h>

#include <control_msgs/JointTrajectoryControllerState.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

namespace sim_control
{
	std::atomic<bool> callback_called = false;

	constexpr std::array<double, 7> VEC_INIT_JOINT_POS = { 0, -0.785, 0, -2.356, 0, 1.571, 0.785 };
	const     std::string CONTROLLER_NAME = "panda_joint_trajectory_controller";

	control_msgs::JointTrajectoryControllerState msg_joint_traj_state;
	std::mutex                                   mtx_joint_traj_state;

	void
	callback_joint_traj_state(const control_msgs::JointTrajectoryControllerStateConstPtr& msg)
	{
		callback_called = true;

		std::lock_guard lock(mtx_joint_traj_state);
		msg_joint_traj_state = *msg;
	}

	bool start_controller()
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
	ros::init(argc, argv, "joint_trajectory_control");
	ros::NodeHandle nh;

	// start 'panda_joint_trajectory_controller' when controller_manager is loaded
	sim_control::start_controller();

	// wait for gazebo + controllers to launch
	ros::service::waitForService("/" + sim_control::CONTROLLER_NAME + "/query_state");

	// subsribers
	auto sub_joint_traj_state = nh.subscribe<control_msgs::JointTrajectoryControllerState>(sim_control::CONTROLLER_NAME + "/state", 100, sim_control::callback_joint_traj_state);
	auto pub_joint_traj_state = nh.advertise<trajectory_msgs::JointTrajectory>(sim_control::CONTROLLER_NAME + "/command", 100);

	// start async callbacks
	auto spinner = ros::AsyncSpinner(0);
	spinner.start();

	// wait (block) for callback to be called
	while (ros::ok && not sim_control::callback_called)
		;

	// log info
	ROS_INFO("[OK] Initialized '%s' node.", ros::this_node::getName().c_str());

	// construct initial robot state (trajectory pts + trajectory)
	trajectory_msgs::JointTrajectory joint_traj;
	trajectory_msgs::JointTrajectoryPoint joint_traj_pt;

	for (size_t i = 0; i < sim_control::VEC_INIT_JOINT_POS.size(); ++i)
	{
		auto joint_name = "panda_joint" + std::to_string(i + 1);
		ROS_WARN("Adding joint: '%s'", joint_name.c_str());

		joint_traj_pt.positions.push_back(sim_control::VEC_INIT_JOINT_POS[i]);
		joint_traj.joint_names.push_back(joint_name);
	}

	joint_traj_pt.time_from_start = ros::Duration(2);
	joint_traj.points.push_back(joint_traj_pt);

	// get user command to execute trajectory control
	std::cout << "\n\nPress [ENTER] to execute trajectory...\n";
	std::cin.get();
	pub_joint_traj_state.publish(joint_traj);

	// main loop
	ros::Rate rate(1); // hz
	while (ros::ok)
	{
		using namespace sim_control;

		std::lock_guard lock(sim_control::mtx_joint_traj_state);

		// read and output actual + desired joint states
		ROS_WARN("Reading joint states:");

		for (size_t i = 0; i < msg_joint_traj_state.joint_names.size(); ++i)
			ROS_INFO_STREAM(msg_joint_traj_state.joint_names[i] << " | " << msg_joint_traj_state.actual.positions[i] << " | " << msg_joint_traj_state.desired.positions[i]);

		std::cout << "\n" << std::flush;

		// sleep for remainding time
		rate.sleep();
	}

	// exit
	ros::waitForShutdown();
	return 0;
}