#include "pd_controller.h"

#include <iomanip>
#include <iostream>
#include <vector>

#include <franka_dynamics/franka_dynamics.h>

#include <Eigen/Core>

void
PDController::computeControls(const SimTK::State& state, SimTK::Vector& vec_controls) const
{
	// > control loop
	// this method is called at every time step for every actuator

	// get current time in simulation
	auto t = state.getTime();

	// get joint actuators
	// lambda to access joints actuators, e.g. Q(0) -> joint0
	auto Q = [&](size_t i) { return (const OpenSim::CoordinateActuator&)(this->getActuatorSet()[i]); };

	// get vector of joint positions
	auto vec_q    = state.getQ();
	auto vec_qdot = state.getQDot();

	// convert SimTK::Vector(s) to Eigen::Vector(s)
	static Eigen::Vector7d q = Eigen::Vector7d::Zero(), qdot = Eigen::Vector7d::Zero();
	for (size_t i = 0; i < vec_q.size(); ++i)
	{
		q[i]    = vec_q[i];
		qdot[i] = vec_qdot[i];
	}

	// compute dynamics
	auto M = franka::dynamics::mass(q);
	//auto C = franka::dynamics::coriolis(q, qdot);
	auto g = franka::dynamics::gravity(q);
	//auto f = franka::dynamics::friction(q);

	// calculate mass matrix using SimBody engine
	/*auto& sys = this->getModel().getMultibodySystem().getMatterSubsystem();
	SimTK::Matrix mat;
	sys.calcM(state, mat);*/
	//std::cout << "sys.getNumBodies(): " << sys.getNumBodies() << std::endl;
	//std::cout << mat << "\n";

	// desired joint positions
	static Eigen::Vector7d q_d;
	q_d << 0, -0.785, 0, -2.356, 0, 1.571, 0.785;

	// control equation (desired torque)
	auto u = kp * (q_d - q) - kd * qdot + g;

	// std::cout << "q   : " << q << "\n\n";
	// std::cout << "qdot: " << qdot << "\n\n";
	// std::cout << "M:\n" << M << "\n\n";
	// std::cout << "g:\n" << g << "\n\n";
	// std::cout << "u: " << u << "\n\n";
	// std::cout << std::endl;

	// apply desired torques
	static SimTK::Vector des_torque(1, 0.0);
	for (size_t i = 0; i < u.size(); ++i)
	{
		des_torque[0] = u[i];
		Q(i).addInControls(des_torque, vec_controls);
	}
}