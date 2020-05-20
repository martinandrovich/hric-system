#pragma once

#include <OpenSim/OpenSim.h>
#include <OpenSim/Simulation/Control/Controller.h>

class PDController : public OpenSim::Controller {
OpenSim_DECLARE_CONCRETE_OBJECT(PDController, Controller);

public:

	PDController(double kp, double kd) : Controller(), kp(kp), kd(kd) {}

	void
	computeControls(const SimTK::State& state, SimTK::Vector &vec_controls) const override
	{
		// > control loop
		// this method is called at every time step for every actuator

		// current time in simulation
		auto t = state.getTime();

		// get joint actuators
		// lambda to access joints, e.g. q(0) -> joint0
		// auto q = [&](size_t i){ return (const OpenSim::CoordinateActuator&)(this->getActuatorSet()[i]); };

		auto q0 = (const OpenSim::CoordinateActuator&)(this->getActuatorSet()[0]);

		// vector of desied torques
		std::vector<double> tau_des = { 0.0, 0.0 };

		SimTK::Vector vec_desired(1, 10.0);
		q0.addInControls(vec_desired, vec_controls);

	}

private:

	double kp;
	double kd;

};
