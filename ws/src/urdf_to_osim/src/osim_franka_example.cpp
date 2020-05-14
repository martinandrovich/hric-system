
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

	// ----------------------------------------------------------------------------------------------

	// create OSIM model
	OpenSim::Model model;
	std::string model_name = "panda";
	model.setName(model_name);
	model.setAuthors("Martin Androvich");

	// ----------------------------------------------------------------------------------------------

	// > links

	OpenSim::Body* link0  = new OpenSim::Body("link0", 1.92, Vec3(-0.025566, -2.88e-05, 0.057332), Inertia(0));
	OpenSim::Body* link1  = new OpenSim::Body("link1", 4.970684, Vec3(0, -0.0324958, -0.0675818), Inertia(0.551121276649726, 0.554308164573063, 0.009020836222789, -9.891702555950000e-05, 0.003401254912500, 0.017358801154300));
	OpenSim::Body* link2  = new OpenSim::Body("link2", 0.646926, Vec3(0, -0.06861, 0.0322285), Inertia(0));
	OpenSim::Body* link3  = new OpenSim::Body("link3", 3.228604, Vec3(0.0469893, 0.0316374, -0.031704), Inertia(0));

	// ----------------------------------------------------------------------------------------------

	// > joints

	bool fix_orientation = true;
	auto vec_ground_ori = (fix_orientation) ? Vec3(-M_PI/2, 0, 0) : Vec3(0);

	// world_to_link0 joint
	OpenSim::WeldJoint* world_to_link0 = new OpenSim::WeldJoint
	(
		"world_to_link0",                                 // joint name
		model.getGround(), Vec3(0), vec_ground_ori,       // parent body, location in parent, orientation in parent
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

	// link0 visual
	OpenSim::Mesh link0_visual("meshes/visual/link0.stl");
	link0->attachGeometry(link0_visual.clone());

	// link1 visual
	OpenSim::Mesh link1_visual("meshes/visual/link1.stl");
	link1->attachGeometry(link1_visual.clone());

	// link2 visual
	OpenSim::Mesh link2_visual("meshes/visual/link2.stl");
	link2->attachGeometry(link2_visual.clone());

	// link3 visual
	OpenSim::Mesh link3_visual("meshes/visual/link3.stl");
	link3->attachGeometry(link3_visual.clone());

	// ----------------------------------------------------------------------------------------------

	// > contact geometry

	// for the joints, BushingForce should be used to define damping
	// https://simtk-confluence.stanford.edu:8443/display/OpenSim33/OpenSim+Models#OpenSimModels-Forces

	// contact parameters
	double stiffness = 1e7, dissipation = 0.1, fric_static = 1.05, fric_dynamic = 1.4, fric_viscous = 1.0;
	auto contact_params = new OpenSim::ElasticFoundationForce::ContactParameters(stiffness, dissipation, fric_static, fric_dynamic, fric_viscous);

	// default contact geometry appearance
	auto contact_appearance = new OpenSim::Appearance();
	contact_appearance->set_visible(false);

	// link0 contact mesh
	auto link0_contact = new ContactMesh("meshes/collision/link0.stl", Vec3(0), Vec3(0), *link0, "link0_contact");
	model.addContactGeometry(link0_contact);

	// link1 contact mesh
	auto link1_contact = new ContactMesh("meshes/collision/link1.stl", Vec3(0), Vec3(0), *link1, "link1_contact");
	model.addContactGeometry(link1_contact);

	// link2 contact mesh
	// auto link2_contact = new ContactMesh("meshes/collision/link2.stl", Vec3(0), Vec3(0), *link2, "link2_contact");
	// model.addContactGeometry(link2_contact);

	// link3 contact mesh
	auto link3_contact = new ContactMesh("meshes/collision/link3.stl", Vec3(0), Vec3(0), *link3, "link3_contact");
	model.addContactGeometry(link3_contact);

	// add contact forces
	auto contact_force = new OpenSim::ElasticFoundationForce(contact_params);
	contact_force->setName("contact_force");

	contact_force->addGeometry("link0_contact");
	contact_force->addGeometry("link1_contact");
	// contact_force->addGeometry("link2_contact");
	contact_force->addGeometry("link3_contact");

	model.addForce(contact_force);

	// ----------------------------------------------------------------------------------------------

	// > actuators
	
	// joint0 actuator
	auto joint0_actuator = new OpenSim::CoordinateActuator(joint0->getCoordinate().getName());
	model.addModelComponent(joint0_actuator);

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
	auto path_model = boost::filesystem::current_path().string() + "/" + model_name + ".osim";
	ROS_INFO("Exporting model to: '%s'", path_model.c_str());
	model.print(path_model);
	
	// ----------------------------------------------------------------------------------------------

	// exit
	return 0;
}