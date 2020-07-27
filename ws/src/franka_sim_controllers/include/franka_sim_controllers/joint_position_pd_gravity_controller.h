#pragma once

#include <string>
#include <vector>

#include <ros/ros.h>
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <realtime_tools/realtime_buffer.h>
#include <std_msgs/Float64MultiArray.h>

#include <franka_gazebo/dynamics.h>
#include <Eigen/Core>

namespace franka_sim_controllers
{
class JointPositionPDGravityController final
	: public controller_interface::Controller<hardware_interface::EffortJointInterface>
{

public:

	static inline constexpr auto CONTROLLER_NAME  = "JointPositionPDGravityController";
	static inline constexpr auto SATURATE_ROTATUM = true;
	static inline constexpr auto TAU_DOT_MAX      = 1000.;

	std::vector<std::string> vec_joint_names;
	size_t num_joints;

	std::vector<hardware_interface::JointHandle> vec_joints;
	realtime_tools::RealtimeBuffer<std::vector<double>> commands_buffer;

	JointPositionPDGravityController() {}
	~JointPositionPDGravityController() { sub_command.shutdown(); }

	bool
	init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle& nh) override;

	void
	starting(const ros::Time& time) override;

	void 
	update(const ros::Time& /*time*/, const ros::Duration& /*period*/) override;

private:

	ros::Subscriber sub_command;
	ros::Subscriber sub_joint_state;

	double kp = 100.0;
	double kd =  50.0;

	Eigen::VectorXd q_d;

	Eigen::Vector7d
	get_position();

	Eigen::Vector7d
	get_velocity();

	Eigen::Vector7d
	saturate_rotatum(const Eigen::Vector7d& tau_des, const double period = 0.001 /* [s] */);

	void
	callback_command(const std_msgs::Float64MultiArrayConstPtr& msg);

};
}