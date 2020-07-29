#pragma once

#include <array>
#include <memory>
#include <string>
#include <mutex>

#include <ros/time.h>
#include <ros/node_handle.h>
#include <hardware_interface/robot_hw.h>
#include <controller_interface/multi_interface_controller.h>
#include <geometry_msgs/Point.h>

#include <franka_hw/franka_state_interface.h>
#include <franka_hw/franka_cartesian_command_interface.h>

#include <Eigen/Core>
#include <kdl/velocityprofile_trap.hpp>
#include <kdl/velocityprofile_spline.hpp>

namespace Eigen
{
	using Matrix7d = Eigen::Matrix<double, 7, 7>;
	using Vector7d = Eigen::Matrix<double, 7, 1>;
}

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
	update(const ros::Time& /*time*/, const ros::Duration& period) override;

private:

	void
	generate_trajectory(const geometry_msgs::Point& pos, bool relative = false);

	void
	generate_mocap_trajectory(const geometry_msgs::Point& pos);

	void
	callback_command_rel(const geometry_msgs::PointConstPtr& msg);
	
	void
	callback_command_abs(const geometry_msgs::PointConstPtr& msg);

	void
	callback_mocap_marker(const geometry_msgs::PointConstPtr& msg);

	franka_hw::FrankaPoseCartesianInterface* cartesian_pose_interface;
	std::unique_ptr<franka_hw::FrankaCartesianPoseHandle> cartesian_pose_handle;

	ros::Duration elapsed_time;
	std::array<double, 16> pose_cur{};

	ros::Subscriber sub_command_abs;
	ros::Subscriber sub_command_rel;
	ros::Subscriber sub_mocap_marker;

	std::mutex mutex_robot_state;

	Eigen::Vector3d EE_pos_des;
	Eigen::Vector3d EE_pos_cur;
	Eigen::Vector3d EE_pos_prev;
	Eigen::Vector3d EE_vel;
	Eigen::Vector3d EE_vel_prev;
	Eigen::Vector3d EE_acc;

	Eigen::Vector7d qdot;
	Eigen::Vector7d qddot;

	static constexpr auto MAX_VELOCITY     =  0.5; // [m/s]
	static constexpr auto MAX_ACCELERATION =  0.1; // [m/s^2]
	static constexpr auto DUR_BUFFER       = 0.00; // [s]
	static constexpr auto MIN_DIST         = 1e-4; // [m]

	KDL::VelocityProfile_Trap vel_profile[3];
	KDL::VelocityProfile_Trap vel_profile_trap[3];


	bool executing_trajectory = false;
	double traj_time = 0.;
	double traj_dur  = 0.;

};
}
