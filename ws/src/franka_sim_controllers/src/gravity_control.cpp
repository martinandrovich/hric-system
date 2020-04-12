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

	inline const auto CONTROLLER_NAME = std::string("panda_joint_effort_controller");
	constexpr auto    VEC_INIT_JOINT_POS = std::array<double, 7>{ 0, -0.785, 0, -2.356, 0, 1.571, 0.785 };

	constexpr auto    PID_FREQ = 100;
	constexpr auto    KP = 10.0;
	constexpr auto    KD = 1.0;
	constexpr auto    dt = 1. / PID_FREQ;

	inline double     kp, kd;

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

	using namespace sim_control;
	using namespace franka_gazebo;

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
	tau_d.data.resize(dynamics::NUM_JOINTS);

	// desired position (q)
	Eigen::VectorXd q_d(7);
	for (size_t i = 0; i < VEC_INIT_JOINT_POS.size(); ++i)
		q_d[i] = VEC_INIT_JOINT_POS[i];

	// log info
	ROS_INFO("[OK] Initialized '%s' node.", ros::this_node::getName().c_str());

	// wait for user input
	std::cout << "\nPress [ENTER] to start controller...\n";
	std::cin.get();

	// gravity controller PID loop
	ros::Rate rate(PID_FREQ); // hz
	while (ros::ok)
	{
		// read PID parameters from ROS parameter server; if fails, set to default values
		if (not ros::param::get("/gravity_control/pid/kp", kp))
			kp = KP;

		if (not ros::param::get("/gravity_control/pid/kd", kd))
			kd = KD;

		// get measurements
		const auto q    = dynamics::position().data;
		const auto qdot = dynamics::velocity().data;
		const auto G    = dynamics::gravity().data;

		// effort command (torque)
		const auto u = kp * (q_d - q) - kd * qdot + G;

		// construct torque command
		for (size_t i = 0; i < dynamics::NUM_JOINTS; ++i)
		{
			tau_d.data[i] = u[i];
			ROS_INFO("tau_d[%lu]: %f", i, tau_d.data[i]);
		}

		// publish desired torques
		pub_tau_d.publish(tau_d);

		// sleep for remainding time
		rate.sleep();
	}

	ROS_WARN("EXITING...");

	// exit
	spinner.stop();
	ros::shutdown();
	ros::waitForShutdown();
	return 0;
}