#include "pd_controller.h"

#include <iomanip>
#include <iostream>
#include <vector>

void
PDController::computeControls(const SimTK::State& state, SimTK::Vector& vec_controls) const
{
	// > control loop
	// this method is called at every time step for every actuator

	// get current time in simulation
	auto t = state.getTime();

	// get joint actuators
	// lambda to access joints actuators, e.g. q(0) -> joint0
	auto q = [&](size_t i) { return (const OpenSim::CoordinateActuator&)(this->getActuatorSet()[i]); };
	//auto q0 = (const OpenSim::CoordinateActuator&)(this->getActuatorSet()[0]);

	// vector of desied torques
	std::vector<double> vec_tau_des = { 10.0 };

	// assert that number of desired torques = num joints
	if (vec_tau_des.size() != vec_controls.size())
	{
		std::cerr << "Size of desired torques [" << vec_tau_des.size() << "] and number of controls [" << vec_controls.size() << "] does not match!\n";
		return;
	}

	// get vector of joint positions
	auto vec_q = state.getQ();
	std::cout << "q: " << vec_q << "\n";

	// calculate mass matrix
	auto& sys = this->getModel().getMultibodySystem().getMatterSubsystem();
	SimTK::Matrix mat;
	sys.calcM(state, mat);
	//std::cout << "sys.getNumBodies(): " << sys.getNumBodies() << std::endl;
	//std::cout << mat << "\n";

	// apply desired torques
	SimTK::Vector des_torque(1, 0.0);
	for (size_t i = 0; i < vec_tau_des.size(); ++i)
	{
		des_torque[0] = vec_tau_des[i];
		q(i).addInControls(des_torque, vec_controls);
	}
}