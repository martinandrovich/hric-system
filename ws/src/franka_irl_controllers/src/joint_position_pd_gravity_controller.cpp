#include <franka_irl_controllers/joint_position_pd_gravity_controller.h>
#include <pluginlib/class_list_macros.hpp>

namespace franka_irl_controllers
{

bool
JointPositionPDGravityController::init(hardware_interface::RobotHW* robot_hardware, ros::NodeHandle& node_handle)
{
	// get effort joint interface
	effort_joint_interface = robot_hardware->get<hardware_interface::EffortJointInterface>();

	// get arm id
	if (!node_handle.getParam("arm_id", arm_id))
	{
		ROS_ERROR("The arm id was not specified");
		return false;
	}

	// get list of joints (from parameter server)
	if (!node_handle.getParam("joint_names", vec_joint_names))
	{
		ROS_ERROR("No joints were specifed");
		return false;
	}

	// get number of joints; exit if zero
	if (num_joints = vec_joint_names.size(); num_joints == 0)
	{
		ROS_ERROR("Vector of joint names is empty");
		return false;
	}

	// fill vector of joint handles
	for (const auto& joint_name : vec_joint_names)
	{
		try
		{
			vec_joints.push_back(effort_joint_interface->getHandle(joint_name));
		}
		catch (const hardware_interface::HardwareInterfaceException& e)
		{
			ROS_ERROR("Error getting joint handles: %s", e.what());
			return false;
		}
	}

	// get franka model interface
	franka_model_interface = robot_hardware->get<franka_hw::FrankaModelInterface>();
	if (franka_model_interface == nullptr)
	{
		ROS_ERROR_STREAM("Error getting model interface from hardware");
		return false;
	}

	// get franka model interface handle
	try
	{
		franka_model_handle = std::make_unique<franka_hw::FrankaModelHandle>(franka_model_interface->getHandle(arm_id + "_model"));
	}
	catch (hardware_interface::HardwareInterfaceException& ex)
	{
		ROS_ERROR_STREAM("Exception getting model handle from interface: " << ex.what());
		return false;
	}

	// get franka state interface
	franka_state_interface = robot_hardware->get<franka_hw::FrankaStateInterface>();
	if (franka_state_interface == nullptr)
	{
		ROS_ERROR_STREAM("Error getting state interface from hardware");
		return false;
	}

	// get franka state interface handle
	try
	{
		franka_state_handle = std::make_unique<franka_hw::FrankaStateHandle>(franka_state_interface->getHandle(arm_id + "_robot"));
	}
	catch (hardware_interface::HardwareInterfaceException& ex)
	{
		ROS_ERROR_STREAM("Exception getting state handle from interface: " << ex.what());
		return false;
	}

	// subscribe to joint position command
	sub_command = node_handle.subscribe<std_msgs::Float64MultiArray>("command", 1, &JointPositionPDGravityController::callback_command, this);

	// init complete
	ROS_INFO_STREAM("Loaded " << "JointPositionPDGravityController" << " with kp = " << kp << ", kd = " << kd);
	return true;
}

void
JointPositionPDGravityController::starting(const ros::Time& time)
{
	// initial desired position
	q_d = (Eigen::VectorXd(7) << 0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4).finished();
	// q_d = (Eigen::VectorXd(7) << 0, 0, 0, 0, 0, 0, 0).finished();

	const std::vector<double>& commands = *commands_buffer.readFromRT();
	for (size_t i = 0; i < commands.size(); ++i)
		q_d[i] = commands[i];

	// init elapsed time
	elapsed_time = ros::Duration(0.);
}

void 
JointPositionPDGravityController::update(const ros::Time& /*time*/, const ros::Duration& period)
{
	// read commands (get get desired joint efforts)
	const std::vector<double>& commands = *commands_buffer.readFromRT();
	for (size_t i = 0; i < commands.size(); ++i)
		q_d[i] = commands[i];
	
	// update elapsed time
	elapsed_time += period;
	
	// read joint states
	const auto q    = get_position();
	const auto qdot = get_velocity();

	// read joint dynamics
	const auto g = franka_model_handle->getGravity();

	// read robot state
	franka::RobotState robot_state = franka_state_handle->getRobotState();
	Eigen::Map<Eigen::Matrix<double, 7, 1>> tau_J_d(robot_state.tau_J_d.data());

	// compute controller effort (desired torque)
	Eigen::Vector7d tau_des = kp * (q_d - q) - kd * qdot;

	// saturate rate of torque (rotatum)
	if (SATURATE_ROTATUM)
		tau_des = saturate_rotatum(tau_des, period.toSec(), tau_J_d);

	// set desired command on joint handles
	for (size_t i = 0; i < num_joints; ++i)
		vec_joints[i].setCommand(tau_des[i]);
}

Eigen::Vector7d
JointPositionPDGravityController::get_position()
{
	static Eigen::Vector7d q;

	for (size_t i = 0; i < vec_joints.size(); ++i)
		q[i] = vec_joints[i].getPosition();

	return q;
}

Eigen::Vector7d
JointPositionPDGravityController::get_velocity()
{
	static Eigen::Vector7d qdot;

	for (size_t i = 0; i < vec_joints.size(); ++i)
		qdot[i] = vec_joints[i].getVelocity();

	return qdot;
}

Eigen::Vector7d
JointPositionPDGravityController::saturate_rotatum(const Eigen::Vector7d& tau_des, const double period, const Eigen::Vector7d& tau_des_prev)
{
	// previous desired torque and saturated torque
	// static Eigen::Vector7d tau_des_prev = Eigen::Vector7d::Zero();
	static Eigen::Vector7d tau_des_sat  = Eigen::Vector7d::Zero();
	
	// compute saturated torque
	for (size_t i = 0; i < tau_des_sat.size(); ++i)
	{
		const double diff = tau_des[i] - tau_des_prev[i];
		tau_des_sat[i] = tau_des_prev[i] + std::max(std::min(diff, 1.0), -1.0);
		// const double tau_dot = (tau_des[i] - tau_des_prev[i]) / period;
		// tau_des_sat[i] = tau_des_prev[i] + std::max(std::min(tau_dot, TAU_DOT_MAX * period), -(TAU_DOT_MAX * period));
	}

	// save for next iteration and return
	// tau_des_prev = tau_des_sat;
	return tau_des_sat;
}

void
JointPositionPDGravityController::callback_command(const std_msgs::Float64MultiArrayConstPtr& msg)
{
	// check size
	if (msg->data.size() != num_joints)
	{
		ROS_ERROR("Number of desired values in command (%lu) does not match number of joints (%lu); execution aborted.", msg->data.size(), num_joints);
		return;
	}

	// write commands to command buffer
	commands_buffer.writeFromNonRT(msg->data);
}

}

// export controller
PLUGINLIB_EXPORT_CLASS(franka_irl_controllers::JointPositionPDGravityController, controller_interface::ControllerBase)