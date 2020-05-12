
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
	auto model_name = "panda";
	model.setAuthors("Martin Androvich");
	model.setName(model_name);

	// ----------------------------------------------------------------------------------------------

	// > links

	OpenSim::Body* link0  = new OpenSim::Body("link0", 2.92, Vec3(-0.025566, -2.88e-05, 0.057332), Inertia(0));
	OpenSim::Body* link1  = new OpenSim::Body("link1", 2.74, Vec3(0, -0.0324958, -0.0675818), Inertia(0));
	OpenSim::Body* link2  = new OpenSim::Body("link2", 2.74, Vec3(0, -0.06861, 0.0322285), Inertia(0));
	OpenSim::Body* link3  = new OpenSim::Body("link3", 2.38, Vec3(0.0469893, 0.0316374, -0.031704), Inertia(0));

	// ----------------------------------------------------------------------------------------------

	// > joints

	// world_to_link0 joint
	OpenSim::WeldJoint* world_to_link0 = new OpenSim::WeldJoint
	(
		"world_to_link0",                                 // joint name
		model.getGround(), Vec3(0), Vec3(-M_PI/2, 0, 0),  // parent body, location in parent, orientation in parent
		*link0, Vec3(0, 0, 0), Vec3(0, 0, 0)              // child body, location in child, orientation in child
	);

	// joint0 (link0 <--> link1)
	OpenSim::PinJoint* joint0 = new OpenSim::PinJoint
	(
		"joint0",                                         // joint name
		*link0, Vec3(0, 0, 0.333), Vec3(0),               // parent body, location in parent, orientation in parent
		*link1, Vec3(0, 0, 0), Vec3(0, 0, 0)              // child body, location in child, orientation in child
	);

	// joint1 (link1 <--> link2)
	OpenSim::PinJoint* joint1 = new OpenSim::PinJoint
	(
		"joint1",                                         // joint name
		*link1, Vec3(0, 0, 0), Vec3(-M_PI/2, 0, 0),       // parent body, location in parent, orientation in parent
		*link2, Vec3(0, 0, 0), Vec3(0, 0, 0)              // child body, location in child, orientation in child
	);

	// joint1 (link2 <--> link3)
	OpenSim::PinJoint* joint2 = new OpenSim::PinJoint
	(
		"joint2",                                         // joint name
		*link2, Vec3(0, -0.316, 0), Vec3(M_PI/2, 0, 0),   // parent body, location in parent, orientation in parent
		*link3, Vec3(0, 0, 0), Vec3(0, 0, 0)              // child body, location in child, orientation in child
	);

	// ----------------------------------------------------------------------------------------------

	// > display geometry

	// link0 mesh
	OpenSim::Mesh link0_mesh("meshes/visual/link0.stl");
	link0->attachGeometry(link0_mesh.clone());

	// link1 mesh
	OpenSim::Mesh link1_mesh("meshes/visual/link1.stl");
	link1->attachGeometry(link1_mesh.clone());

	// link2 mesh
	OpenSim::Mesh link2_mesh("meshes/visual/link2.stl");
	link2->attachGeometry(link2_mesh.clone());

	// link3 mesh
	OpenSim::Mesh link3_mesh("meshes/visual/link3.stl");
	link3->attachGeometry(link3_mesh.clone());

	// ----------------------------------------------------------------------------------------------

	// add components to model

	model.addBody(link0);
	model.addBody(link1);
	model.addBody(link2);
	model.addBody(link3);

	model.addJoint(world_to_link0);
	model.addJoint(joint0);
	model.addJoint(joint1);
	model.addJoint(joint2);

	// ----------------------------------------------------------------------------------------------

	// finalize connections
	model.finalizeConnections();

	// export model
	auto path_model = dir_osim + model_name + ".osim";
	ROS_INFO("Exporting model to: '%s'", path_model.c_str());
	model.print(path_model);
	
	// ----------------------------------------------------------------------------------------------

	// exit
	return 0;
}