#include <franka_sim_controllers/joint_position_pd_gravity_controller.h>
#include <pluginlib/class_list_macros.hpp>

#include <franka_gazebo/logger.h>

namespace franka_sim_controllers
{

bool
JointPositionPDGravityController::init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle& nh)
{
	// get list of joints (from parameter server)
	if (!nh.getParam("joint_names", vec_joint_names))
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

	// initialize franka panda dynamics library
	// subscription unnecesary since the direct gravity() method is used in control loop
	// sub_joint_state = nh.subscribe<sensor_msgs::JointState>("/joint_states", 100, franka_gazebo::dynamics::callback_joint_state);
	franka_gazebo::dynamics::init();

	// init complete
	ROS_INFO_STREAM_NAMED(CONTROLLER_NAME, "Loaded " << CONTROLLER_NAME << " with kp = " << kp << ", kd = " << kd);
	return true;
}

void
JointPositionPDGravityController::starting(const ros::Time& time)
{
	// start controller with 0 efforts
	commands_buffer.readFromRT()->assign(num_joints, 0.);

	// initial desired position
	q_d = (Eigen::VectorXd(7) << 0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4).finished();
}

void 
JointPositionPDGravityController::update(const ros::Time& /*time*/, const ros::Duration& period)
{
	// !!!
	// control loop is not optimal with dynamic allocations etc.

	// elapsed time
	static ros::Duration elapsed_time = ros::Duration(0.);
	elapsed_time += period;

	// read commands (get get desired joint efforts)
	const std::vector<double>& commands = *commands_buffer.readFromRT();

	// read joint states
	const auto q    = get_position();
	const auto qdot = get_velocity();

	// compute dynamics
	const auto g    = franka_gazebo::dynamics::gravity(q);

	// compute controller effort
	Eigen::Vector7d tau_des = kp * (q_d - q) - kd * qdot + g;

	// saturate rate-of-effort (rotatum)
	if (SATURATE_ROTATUM)
		tau_des = saturate_rotatum(tau_des, period.toSec());

	// set desired command on joint handles
	for (size_t i = 0; i < num_joints; ++i)
		vec_joints[i].setCommand(tau_des[i]);

	// log data
	log_panda_info("test", elapsed_time.toSec(), q, qdot, tau_des, g);
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
JointPositionPDGravityController::saturate_rotatum(const Eigen::Vector7d& tau_des, const double period)
{
	// previous desired torque and saturated torque
	static Eigen::Vector7d tau_des_prev = Eigen::Vector7d::Zero();
	static Eigen::Vector7d tau_des_sat  = Eigen::Vector7d::Zero();
	
	// compute saturated torque
	for (size_t i = 0; i < tau_des_sat.size(); ++i)
	{
		// const double diff = tau_des[i] - tau_des_prev[i];
		// tau_des_sat[i] = tau_des_prev[i] + std::max(std::min(diff, 1.0), -1.0);
		const double tau_dot = (tau_des[i] - tau_des_prev[i]) / period;
		tau_des_sat[i] = tau_des_prev[i] + std::max(std::min(tau_dot, TAU_DOT_MAX * period), -(TAU_DOT_MAX * period));
	}

	// save for next iteration and return
	tau_des_prev = tau_des_sat;
	return tau_des_sat;
}

void
JointPositionPDGravityController::callback_command(const std_msgs::Float64MultiArrayConstPtr& msg)
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

}

// export controller
PLUGINLIB_EXPORT_CLASS(franka_sim_controllers::JointPositionPDGravityController, controller_interface::ControllerBase)