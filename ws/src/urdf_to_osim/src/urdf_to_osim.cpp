#include "urdf_to_osim/urdf_to_osim.h"

#include <OpenSim/OpenSim.h>

bool
urdf_to_osim(const std::string& path_to_urdf)
{
	ROS_INFO("Constructing KDL Tree from URDF model (%s).", path_to_urdf.c_str());
	
	// create KDL tree from URDF model
	KDL::Tree kdl_tree;
	if (not kdl_parser::treeFromFile(path_to_urdf, kdl_tree))
	{
		ROS_ERROR("Failed to construct KDL tree.");
		return false;
	}

	// testing
	for (const auto [name, element] : kdl_tree.getSegments())
		std::cout << name << " [" << element.q_nr << "]: " << element.segment.getInertia().getMass() << " kg\n";

	// convert KDL tree to OSIM model
	;
	OpenSim::Model model;

	model.setAuthors("testing");

	return true;
}

// ----------------------------------------------------------------------------------------------

int
main(int argc, char** argv)
{

	// verfify that enough arguments are passed
	if (argc != 3)
	{
		ROS_WARN("Not enough parameters supplied.");
		ROS_WARN("Usage: urdf_to_osim <input_path.urdf> <output_path.osim>");
		return 0;
	}

	// get parameters
	auto path_to_urdf = argv[1];
	auto path_to_osim = argv[2];

	// create OSIM model
	auto osim_model = urdf_to_osim(path_to_urdf);

	// export OSIM model to specified path
	;

	return 0;
}