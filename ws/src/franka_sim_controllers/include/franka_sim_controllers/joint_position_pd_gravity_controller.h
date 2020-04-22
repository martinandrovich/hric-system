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

namespace franka_sim_controllers
{
class JointPositionPDGravityController : public controller_interface::Controller<hardware_interface::EffortJointInterface>
{

public:

	static constexpr auto CONTROLLER_NAME = "JointPositionPDGravityController";

	std::vector<std::string> vec_joint_names;
	size_t num_joints;

	std::vector<hardware_interface::JointHandle> vec_joints;
	realtime_tools::RealtimeBuffer<std::vector<double>> commands_buffer;

	JointPositionPDGravityController() {}
	~JointPositionPDGravityController() { sub_command.shutdown(); }

	bool
	init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle& nh)
	{
		// get list of joints (from parameter server)
		if (!nh.getParam("joints", vec_joint_names))
		{
			ROS_ERROR_NAMED(CONTROLLER_NAME, "No joints were specifed.");
			return false;
		}

		// get number of joints; exit if zero
		if (num_joints = vec_joint_names.size(); num_joints == 0)
		{
			ROS_ERROR_NAMED(CONTROLLER_NAME, "Vector of joint names is empty.");
			return false;
		}

		// fill vector of joint handles
		for (const auto& joint_name : vec_joint_names)
		{
			try
			{
				vec_joints.push_back(hw->getHandle(joint_name));
			}
			catch (const hardware_interface::HardwareInterfaceException& e)
			{
				ROS_ERROR_NAMED(CONTROLLER_NAME, "Error getting joint handles: %s", e.what());
				return false;
			}
		}

		// initialize command buffer
		commands_buffer.writeFromNonRT(std::vector<double>(num_joints, 0.0));

		// subscribe to joint position command
		sub_command = nh.subscribe<std_msgs::Float64MultiArray>("command", 1, &JointPositionPDGravityController::callback_command, this);

		// subsribe to robot joint states
		// initialize franka panda dynamics library
		// should be a part of custom hardware interface
		sub_joint_state = nh.subscribe<sensor_msgs::JointState>("/joint_states", 100, franka_gazebo::dynamics::callback_joint_state);
		franka_gazebo::dynamics::init();

		// init complete
		ROS_INFO_STREAM_NAMED(CONTROLLER_NAME, "Initialized controller.");
		return true;
	}

	void
	starting(const ros::Time& time)
	{
		// start controller with 0.0 efforts
		commands_buffer.readFromRT()->assign(num_joints, 0.0);

		// initial desired position
		constexpr auto VEC_INIT_JOINT_POS = std::array<double, 7>{ 0, -0.785, 0, -2.356, 0, 1.571, 0.785 };
		q_d = Eigen::VectorXd(7);
		for (size_t i = 0; i < VEC_INIT_JOINT_POS.size(); ++i)
			q_d[i] = VEC_INIT_JOINT_POS[i];
	}

	void 
	update(const ros::Time& /*time*/, const ros::Duration& /*period*/)
	{
		
		// BAD IMPLEMENTATION !!!
		// dynamic allocation etc.

		// read commands (get get desired joint efforts)
		std::vector<double>& commands = *commands_buffer.readFromRT();

		// PID gains
		static auto kp   = 50.0;
		static auto kd   = 1.0;

		// get measurements
		const auto q    = franka_gazebo::dynamics::position().data;
		const auto qdot = franka_gazebo::dynamics::velocity().data;
		const auto G    = franka_gazebo::dynamics::gravity().data;

		// compute controller effort
		const auto u = kp * (q_d - q) - kd * qdot + G;

		// set desired command on joint handles
		for (size_t i = 0; i < num_joints; ++i)
			vec_joints[i].setCommand(u[i]);
			// vec_joints[i].setCommand(commands[i]);
	}

private:

	ros::Subscriber sub_command;
	ros::Subscriber sub_joint_state;

	Eigen::VectorXd q_d;

	void
	callback_command(const std_msgs::Float64MultiArrayConstPtr& msg)
	{
		// check size
		if (msg->data.size() != num_joints)
		{
			ROS_ERROR_NAMED(CONTROLLER_NAME, "Number of desired values in command (%lu) does not match number of joints (%lu); execution aborted.", msg->data.size(), num_joints);
			return;
		}

		// write commands to command buffer
		commands_buffer.writeFromNonRT(msg->data);
	}

};
}