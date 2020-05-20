#pragma once

#include <OpenSim/Simulation/Control/Controller.h>
#include <OpenSim/Actuators/CoordinateActuator.h>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Model/ModelComponent.h>

#ifdef _WIN32
	#include "osimPluginDLL.h"
#endif

namespace OpenSim {

class PDController : public OpenSim::Controller {
OpenSim_DECLARE_CONCRETE_OBJECT(PDController, Controller);

public:

	PDController() {}
	PDController(double kp, double kd) : Controller(), kp(kp), kd(kd) {}

	void
	computeControls(const SimTK::State& state, SimTK::Vector &vec_controls) const override;

private:

	double kp;
	double kd;

};

}
