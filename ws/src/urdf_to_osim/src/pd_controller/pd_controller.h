#pragma once

#include <OpenSim/Simulation/Control/Controller.h>
#include <OpenSim/Actuators/CoordinateActuator.h>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Model/ModelComponent.h>

#include <simbody/internal/SimbodyMatterSubsystem.h>

#ifdef _WIN32
	#include "osimPluginDLL.h"
#endif

class PDController final : public OpenSim::Controller {
OpenSim_DECLARE_CONCRETE_OBJECT(PDController, Controller);

public:

	PDController() = default;
	PDController(double kp, double kd) : Controller(), kp(kp), kd(kd) {}
	~PDController() = default;
	
	void
	computeControls(const SimTK::State& state, SimTK::Vector& vec_controls) const override;

private:

	double kp = 50.0;
	double kd = 1.0; 

};