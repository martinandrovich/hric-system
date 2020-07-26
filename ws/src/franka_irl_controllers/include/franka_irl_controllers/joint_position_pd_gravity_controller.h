#pragma once

#include <array>
#include <vector>
#include <string>
#include <memory>
#include <mutex>

#include <ros/time.h>
#include <ros/node_handle.h>
#include <hardware_interface/robot_hw.h>
#include <hardware_interface/joint_command_interface.h>
#include <controller_interface/multi_interface_controller.h>
#include <realtime_tools/realtime_buffer.h>
#include <std_msgs/Float64MultiArray.h>

#include <franka_hw/franka_model_interface.h>
// #include <franka_hw/franka_state_interface.h>
// #include <franka_hw/franka_cartesian_command_interface.h>

#include <Eigen/Core>

namespace Eigen
{
	using Matrix7d = Eigen::Matrix<double, 7, 7>;
	using Vector7d = Eigen::Matrix<double, 7, 1>;
}

namespace franka_irl_controllers
{
class JointPositionPDGravityController final 
	: public controller_interface::MultiInterfaceController<hardware_interface::EffortJointInterface,
	                                                        franka_hw::FrankaModelInterface>
{

public:

	static inline constexpr auto SATURATE_ROTATUM = true;
	static inline constexpr auto TAU_DOT_MAX      = 1000.;

	JointPositionPDGravityController() {}
	~JointPositionPDGravityController() { sub_command.shutdown(); }

	bool
	init(hardware_interface::RobotHW* robot_hardware, ros::NodeHandle& node_handle) override;

	void
	starting(const ros::Time& time) override;

	void 
	update(const ros::Time& /*time*/, const ros::Duration& period) override;

private:

	hardware_interface::EffortJointInterface* effort_joint_interface;
	franka_hw::FrankaModelInterface* franka_model_interface;
	std::unique_ptr<franka_hw::FrankaModelHandle> franka_model_handle;
	
	size_t num_joints;
	std::string arm_id;
	std::vector<hardware_interface::JointHandle> vec_joints;
	std::vector<std::string> vec_joint_names;

	ros::Subscriber sub_command;
	realtime_tools::RealtimeBuffer<std::vector<double>> commands_buffer; // desired joint pos

	double kp = 10; // 50
	double kd =  1; // 10

	ros::Duration elapsed_time;
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