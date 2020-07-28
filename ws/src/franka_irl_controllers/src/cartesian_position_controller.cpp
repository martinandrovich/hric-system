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

	// subscribe to mocap marker position
	sub_mocap_marker = node_handle.subscribe<geometry_msgs::Point>("/mocap_sampler/marker", 1, &CartesianPositionController::callback_mocap_marker, this);

	return true;
}

void
CartesianPositionController::starting(const ros::Time& /* time */)
{
	// read the robot state
	auto robot_state = cartesian_pose_handle->getRobotState();

	// initialize pose and elapsed time
	pose_cur = robot_state.O_T_EE_d;
	elapsed_time = ros::Duration(0.);

	qdot = Eigen::Map<Eigen::Matrix<double, 7, 1>>(robot_state.dq_d.data());
	qddot = Eigen::Map<Eigen::Matrix<double, 7, 1>>(robot_state.ddq_d.data());

	// set current position
	EE_pos_cur[0] = pose_cur[12]; // x
	EE_pos_cur[1] = pose_cur[13]; // y
	EE_pos_cur[2] = pose_cur[14]; // z

	// set desired position (initial)
	EE_pos_des[0] = pose_cur[12]; // x
	EE_pos_des[1] = pose_cur[13]; // y
	EE_pos_des[2] = pose_cur[14]; // z

	// log current position
	std::cout << "\n\n";
	std::cout << "current position:\n"
	          << "x: " << EE_pos_cur[0] << "\n"
	          << "y: " << EE_pos_cur[1] << "\n"
	          << "z: " << EE_pos_cur[2] << "\n"
	          << std::endl;

	std::cout << "desired position (initial):\n"
	          << "x: " << EE_pos_des[0] << "\n"
	          << "y: " << EE_pos_des[1] << "\n"
	          << "z: " << EE_pos_des[2] << "\n"
	          << std::endl;

	// generate initial trajectory
	for (size_t i = 0; i < 3; ++i)
	{
		vel_profile_trap[i].SetMax(MAX_VELOCITY, MAX_ACCELERATION);
		vel_profile_trap[i].SetProfile(EE_pos_cur[i], EE_pos_cur[i]);
	}
	executing_trajectory = true;
}

void
CartesianPositionController::update(const ros::Time& /* time */, const ros::Duration& period)
{
	// mutex guard
	std::lock_guard lock(mutex_robot_state);
	
	// update elapsed time
	elapsed_time += period;
	traj_time += period.toSec();

	// read robot state
	auto robot_state = cartesian_pose_handle->getRobotState();

	// update current EE position { x, y, z}
	EE_pos_cur = { robot_state.O_T_EE_d[12], robot_state.O_T_EE_d[13], robot_state.O_T_EE_d[14] };

	// get joint state
	qdot = Eigen::Map<Eigen::Matrix<double, 7, 1>>(robot_state.dq_d.data());
	qddot = Eigen::Map<Eigen::Matrix<double, 7, 1>>(robot_state.ddq_d.data());

	// compute EE velocity
	EE_vel = (EE_pos_cur - EE_pos_prev) / period.toSec();
	EE_pos_prev = EE_pos_cur;

	// compute EE acceleration
	EE_acc = (EE_vel - EE_vel_prev) / period.toSec();
	EE_vel_prev = EE_vel;
	
	// update pose
	pose_cur = robot_state.O_T_EE_d;

	// command desired pose from velocity profile
	pose_cur[12] = vel_profile_trap[0].Pos(traj_time);
	pose_cur[13] = vel_profile_trap[1].Pos(traj_time);
	pose_cur[14] = vel_profile_trap[2].Pos(traj_time);

	cartesian_pose_handle->setCommand(pose_cur);
	
	if (executing_trajectory and traj_time > traj_dur)
	{
		executing_trajectory = false;
		ROS_WARN("Finished executing trajectory, took %f [s]", traj_dur);
	}
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
CartesianPositionController::callback_mocap_marker(const geometry_msgs::PointConstPtr& msg)
{
	generate_mocap_trajectory(*msg);
}

void
CartesianPositionController::generate_trajectory(const geometry_msgs::Point& pos, bool relative)
{
	// return if current trajectory unfinished
	if (executing_trajectory)
		return;

	if (qdot.cwiseAbs().maxCoeff() > 10. || qddot.cwiseAbs().maxCoeff() > 10.)
		return;
	
	// mutex guard
	std::lock_guard lock(mutex_robot_state);

	// set desired position (relative or absolute)
	if (relative)
	{
		EE_pos_des[0] = EE_pos_cur[0] + pos.x;
		EE_pos_des[1] = EE_pos_cur[1] + pos.y;
		EE_pos_des[2] = EE_pos_cur[2] + pos.z;
	}
	else
	{
		EE_pos_des[0] = pos.x;
		EE_pos_des[1] = pos.y;
		EE_pos_des[2] = pos.z;
	}

	// update trajectory to new desired position
	// array; 0 = x, 1 = y, 2 = z
	for (size_t i = 0; i < 3; ++i)
	{
		vel_profile[i].SetMax(MAX_VELOCITY, MAX_ACCELERATION);
		vel_profile[i].SetProfile(EE_pos_cur[i], EE_pos_des[i]);
	}

	// compute trajectory duration
	traj_dur = std::max({vel_profile[0].Duration(), vel_profile[1].Duration(), vel_profile[2].Duration()}) + 0.05;

	// reset trajectory time
	traj_time = 0.;
	executing_trajectory = true;
}

void
CartesianPositionController::generate_mocap_trajectory(const geometry_msgs::Point& pos)
{
	// static variables
	static bool is_init = false;
	static auto time_prev = ros::Time::now();
	static Eigen::Vector3d marker_pos_init, marker_pos_cur, marker_pos_prev, marker_vel, EE_pos_init;

	// mutex guard for accessing robot state
	std::lock_guard lock(mutex_robot_state);

	// frame transformation
	// x = x, y = -z, z = y
	// Eigen::Vector3d(pos.x, -pos.z, pos.y);

	// first callback; initialize values
	if (not is_init)
	{
		ROS_WARN("Recieved first marker point; setting initial values");

		marker_pos_init = Eigen::Vector3d(pos.x, -pos.z, pos.y);
		EE_pos_init     = Eigen::Vector3d(EE_pos_cur[0], EE_pos_cur[1], EE_pos_cur[2]);
		marker_pos_prev = marker_pos_init;
		
		time_prev = ros::Time::now();
		is_init = true;

		// do not compute trajectory first time; return instead
		return;
	}

	// current marker position (in EE frame)
	marker_pos_cur = Eigen::Vector3d(pos.x, -pos.z, pos.y);

	// compute marker velocity
	const auto time_diff = (time_prev - ros::Time::now()).toSec();
	marker_vel = (marker_pos_cur - marker_pos_prev) / time_diff; // [m/s]
	
	// update previous marker pos and time
	marker_pos_prev = marker_pos_cur;
	time_prev = ros::Time::now();

	// compute diff in marker position
	const auto EE_pos_des = EE_pos_init + (marker_pos_cur - marker_pos_init);

	// return if current trajectory unfinished
	if (executing_trajectory)
		return;

	// return if joint velocity or acceleration are not settled
	// if (qdot.cwiseAbs().maxCoeff() > 10. || qddot.cwiseAbs().maxCoeff() > 10.)
	// 	return;

	// compute duraction from maximum velocity
	const auto dist = (EE_pos_des - EE_pos_cur).cwiseAbs().maxCoeff();
	const auto dur = dist / MAX_VELOCITY;

	// absolute distance to travel must be > 0.001 [m] = 0.1 [cm], otherwise abort
	if (dist < MIN_DIST)
		return;

	// update trajectory to new desired position
	// array; 0 = x, 1 = y, 2 = z
	for (size_t i = 0; i < 3; ++i)
	{
		vel_profile_trap[i].SetMax(MAX_VELOCITY, MAX_ACCELERATION);
		vel_profile_trap[i].SetProfile(EE_pos_cur[i], EE_pos_des[i]);
		// vel_profile_trap[i].SetProfileDuration(EE_pos_cur[i], EE_pos_des[i], 0.5);
	}

	// compute trajectory duration
	traj_dur = std::max({ vel_profile_trap[0].Duration(),
	                      vel_profile_trap[1].Duration(),
	                      vel_profile_trap[2].Duration()}
	                   ) + DUR_BUFFER;

	// reset trajectory time and initite execution
	traj_time = 0.;
	executing_trajectory = true;
}


}

// export controller
PLUGINLIB_EXPORT_CLASS(franka_irl_controllers::CartesianPositionController, controller_interface::ControllerBase)