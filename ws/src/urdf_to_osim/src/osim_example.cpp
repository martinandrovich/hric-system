
#include <iostream>
#include <fstream>
#include <string>

#include <boost/filesystem.hpp>

#include <ros/ros.h>
#include <ros/package.h>
#include <OpenSim/OpenSim.h>

int
main(int argc, char** argv)
{
	
	using namespace SimTK;
	using namespace OpenSim;

	// deduce directories
	auto dir_pkg  = ros::package::getPath("urdf_to_osim") + "/";
	auto dir_data = dir_pkg + "data/";
	auto dir_osim = dir_data + "osim/";

	// check if necessary directories exist
	if (not boost::filesystem::is_directory(dir_osim))
	{
		ROS_WARN("The directory '%s' does not exist; exiting!", dir_osim.c_str());
		return 0;
	}

	// ----------------------------------------------------------------------------------------------

	// create OSIM model
	OpenSim::Model model;
	auto model_name = "arm";
	model.setAuthors("Martin Androvich");
	model.setName(model_name);

	// ----------------------------------------------------------------------------------------------

	// create two links, each with a mass of 1 kg, center of mass at the body's
	// origin, and moments and products of inertia of zero
	OpenSim::Body* humerus = new OpenSim::Body("humerus", 1, Vec3(0), Inertia(0));
	OpenSim::Body* radius  = new OpenSim::Body("radius", 1, Vec3(0), Inertia(0));

	// ----------------------------------------------------------------------------------------------

	// connect the bodies with pin joints. Assume each body is 1 m long

	// shoulder joint
    OpenSim::PinJoint* shoulder = new OpenSim::PinJoint
	(
		"shoulder",                          // joint name
		model.getGround(), Vec3(0), Vec3(0), // parent body, location in parent, orientation in parent
		*humerus, Vec3(0, 1, 0), Vec3(0)     // child body, location in child, orientation in child
	);

	// elbow joint
	OpenSim::PinJoint* elbow = new OpenSim::PinJoint
	(
		"elbow",                             // joint name
		*humerus, Vec3(0), Vec3(0),          // parent body, location in parent, orientation in parent
		*radius, Vec3(0, 1, 0), Vec3(0)      // child body, location in child, orientation in child
	);

	// ----------------------------------------------------------------------------------------------

	// add components to model

	model.addBody(humerus);
	model.addBody(radius);
	model.addJoint(shoulder);
	model.addJoint(elbow);

	// ----------------------------------------------------------------------------------------------

    // add display geometry

    Ellipsoid bodyGeometry(0.1, 0.5, 0.1);
    bodyGeometry.setColor(Gray);
	
    // attach an ellipsoid to a frame located at the center of each body

    PhysicalOffsetFrame* humerusCenter = new PhysicalOffsetFrame(
        "humerusCenter", *humerus, Transform(Vec3(0, 0.5, 0)));
    humerus->addComponent(humerusCenter);
    humerusCenter->attachGeometry(bodyGeometry.clone());

    PhysicalOffsetFrame* radiusCenter = new PhysicalOffsetFrame(
        "radiusCenter", *radius, Transform(Vec3(0, 0.5, 0)));
    radius->addComponent(radiusCenter);
    radiusCenter->attachGeometry(bodyGeometry.clone());

	// ----------------------------------------------------------------------------------------------

	// finalize the model

	// configure the model
	State& state = model.initSystem();

	// fix the shoulder at its default angle and begin with the elbow flexed
	shoulder->getCoordinate().setLocked(state, true);
	elbow->getCoordinate().setValue(state, 0.5 * Pi);
	model.equilibrateMuscles(state);

	// finalize connections
	model.finalizeConnections();

	// ----------------------------------------------------------------------------------------------

	// export model
	auto path_model = dir_osim + model_name + ".osim";
	ROS_INFO("Exporting model to: '%s'", path_model.c_str());
	model.print(path_model.c_str());

	// exit
	return 0;
}