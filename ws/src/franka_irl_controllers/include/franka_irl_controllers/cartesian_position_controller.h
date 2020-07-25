#pragma once

#include <array>
#include <memory>
#include <string>
#include <mutex>

#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/robot_hw.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <geometry_msgs/Point.h>

#include <franka_hw/franka_state_interface.h>
#include <franka_hw/franka_cartesian_command_interface.h>

#include <kdl/velocityprofile_trap.hpp>


namespace franka_irl_controllers
{
class CartesianPositionController
	: public controller_interface::MultiInterfaceController<franka_hw::FrankaPoseCartesianInterface,
	                                                        franka_hw::FrankaStateInterface>
{

public:

	bool
	init(hardware_interface::RobotHW* robot_hardware, ros::NodeHandle& node_handle) override;

	void
	starting(const ros::Time&) override;

	void
	update(const ros::Time&, const ros::Duration& period) override;

private:

	void
	generate_trajectory(const geometry_msgs::Point& pos, bool relative = false);

	void
	callback_command_rel(const geometry_msgs::PointConstPtr& msg);
	
	void
	callback_command_abs(const geometry_msgs::PointConstPtr& msg);

	franka_hw::FrankaPoseCartesianInterface* cartesian_pose_interface;
	std::unique_ptr<franka_hw::FrankaCartesianPoseHandle> cartesian_pose_handle;

	ros::Duration elapsed_time;
	std::array<double, 16> pose_cur{};

	ros::Subscriber sub_command_abs;
	ros::Subscriber sub_command_rel;

	std::mutex mutex_traj;
	KDL::VelocityProfile_Trap vel_profile[3];
	std::array<double, 3> pos_cur{};
	std::array<double, 3> pos_des{};
	double traj_time = 0.;

};
}
