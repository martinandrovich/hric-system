#include <franka_irl_controllers/cartesian_position_controller.h>

#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>

#include <ros/ros.h>
#include <controller_interface/controller_base.h>
#include <hardware_interface/hardware_interface.h>
#include <pluginlib/class_list_macros.h>

#include <franka_hw/franka_cartesian_command_interface.h>

namespace franka_irl_controllers
{

bool
CartesianPositionController::init(hardware_interface::RobotHW* robot_hardware, ros::NodeHandle& node_handle)
{
	// initialize the controller

	cartesian_pose_interface = robot_hardware->get<franka_hw::FrankaPoseCartesianInterface>();

	if (cartesian_pose_interface == nullptr)
	{
		ROS_ERROR("CartesianPositionController: Could not get Cartesian Pose interface from hardware");
		return false;
	}

	std::string arm_id;
	if (!node_handle.getParam("arm_id", arm_id))
	{
		ROS_ERROR("CartesianPositionController: Could not get parameter arm_id");
		return false;
	}

	try
	{
		cartesian_pose_handle = std::make_unique<franka_hw::FrankaCartesianPoseHandle>(cartesian_pose_interface->getHandle(arm_id + "_robot"));
	}
	catch (const hardware_interface::HardwareInterfaceException& e)
	{
		ROS_ERROR_STREAM("CartesianPositionController: Exception getting Cartesian handle: " << e.what());
		return false;
	}

	auto state_interface = robot_hardware->get<franka_hw::FrankaStateInterface>();
	if (state_interface == nullptr)
	{
		ROS_ERROR("CartesianPositionController: Could not get state interface from hardware");
		return false;
	}

	// subscribe to joint position command (absoulte and relative positions)
	sub_command_abs = node_handle.subscribe<geometry_msgs::Point>("command/abs", 1, &CartesianPositionController::callback_command_abs, this);
	sub_command_rel = node_handle.subscribe<geometry_msgs::Point>("command/rel", 1, &CartesianPositionController::callback_command_rel, this);

	return true;
}

void
CartesianPositionController::starting(const ros::Time& /* time */)
{
	// read the robot state
	const auto robot_state = cartesian_pose_handle->getRobotState();

	// initialize pose and elapsed time
	pose_cur = robot_state.O_T_EE_d;
	elapsed_time = ros::Duration(0.);

	// set current position
	pos_cur[0] = pose_cur[12]; // x
	pos_cur[1] = pose_cur[13]; // y
	pos_cur[2] = pose_cur[14]; // z

	// set desired position (initial)
	pos_des[0] = pose_cur[12]; // x
	pos_des[1] = pose_cur[13]; // y
	pos_des[2] = pose_cur[14]; // z

	// log current position
	std::cout << "\n\n";
	std::cout << "current position:\n"
	          << "x: " << pos_cur[0] << "\n"
	          << "y: " << pos_cur[1] << "\n"
	          << "z: " << pos_cur[2] << "\n"
	          << std::endl;

	std::cout << "desired position (initial):\n"
	          << "x: " << pos_des[0] << "\n"
	          << "y: " << pos_des[1] << "\n"
	          << "z: " << pos_des[2] << "\n"
	          << std::endl;

	// generate initial trajectory
	geometry_msgs::Point pt_init;
	pt_init.x = pos_cur[0];
	pt_init.y = pos_cur[1];
	pt_init.z = pos_cur[2];

	generate_trajectory(pt_init);
}

void
CartesianPositionController::update(const ros::Time& /* time */, const ros::Duration& period)
{
	// mutex guard
	std::lock_guard<std::mutex> lock(mutex_traj);
	
	// update elapsed time
	elapsed_time += period;
	traj_time += period.toSec();

	// read robot state
	const auto robot_state = cartesian_pose_handle->getRobotState();

	// update current position
	pos_cur[0] = robot_state.O_T_EE_d[12]; // x
	pos_cur[1] = robot_state.O_T_EE_d[13]; // y
	pos_cur[2] = robot_state.O_T_EE_d[14]; // z
	
	// command desired pose (pos + ori)
	// auto new_pose = robot_state.O_T_EE_d;
	
	pose_cur[12] = vel_profile[0].Pos(traj_time);
	pose_cur[13] = vel_profile[1].Pos(traj_time);
	pose_cur[14] = vel_profile[2].Pos(traj_time);

	cartesian_pose_handle->setCommand(pose_cur);
}

void
CartesianPositionController::callback_command_abs(const geometry_msgs::PointConstPtr& msg)
{
	// generate trajectory from recieved Pose command
	generate_trajectory(*msg);
}

void
CartesianPositionController::callback_command_rel(const geometry_msgs::PointConstPtr& msg)
{
	// generate trajectory from recieved Pose command
	generate_trajectory(*msg, true);
}

void
CartesianPositionController::generate_trajectory(const geometry_msgs::Point& pos, bool relative)
{
	// velocity profile constants
	constexpr auto MAX_VELOCITY     = 0.5; // [m/s]
	constexpr auto MAX_ACCELERATION = 1.0; // [m/s^2]
	
	// mutex guard
	std::lock_guard<std::mutex> lock(mutex_traj);

	// set desired position (relative or absolute)
	if (relative)
	{
		pos_des[0] = pos_cur[0] + pos.x;
		pos_des[1] = pos_cur[1] + pos.y;
		pos_des[2] = pos_cur[2] + pos.z;
	}
	else
	{
		pos_des[0] = pos.x;
		pos_des[1] = pos.y;
		pos_des[2] = pos.z;
	}

	// update trajectory to new desired position
	// array; 0 = x, 1 = y, 2 = z
	for (size_t i = 0; i < 3; ++i)
	{
		vel_profile[i].SetMax(MAX_VELOCITY, MAX_ACCELERATION);
		vel_profile[i].SetProfile(pos_cur[i], pos_des[i]);
	}

	// reset trajectory time
	traj_time = 0.;
}

}

// export controller
PLUGINLIB_EXPORT_CLASS(franka_irl_controllers::CartesianPositionController, controller_interface::ControllerBase)