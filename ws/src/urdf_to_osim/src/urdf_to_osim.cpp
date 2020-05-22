#include "urdf_to_osim/urdf_to_osim.h"

#include <wordexp.h>
#include <string>
#include <regex>

#include <ros/ros.h>
#include <kdl/kdl.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <urdf/model.h>
#include <boost/filesystem.hpp>
#include <OpenSim/OpenSim.h>

auto
get_link_number(const std::string& str_link)
{
	// https://yunmingzhang.wordpress.com/2018/11/13/regular-expressions-in-c-and-extract-first-integer-in-string/

	if (auto n = str_link.find_first_of("0123456789"); n != std::string::npos)
	{
		std::size_t const m = str_link.find_first_not_of("0123456789", n);
		return stoi(str_link.substr(n, m != std::string::npos ? m-n : m));
	}

	return -1;
}


namespace boost::filesystem {
bool
copy_directory_recursive(const path& source, const path& destination)
{
	// https://stackoverflow.com/questions/8593608/how-can-i-copy-a-directory-using-boost-filesystem

	try
	{
		// check whether the function call is valid
		if(!exists(source) ||!is_directory(source))
		{
			std::cerr << "Source directory " << source.string() << " does not exist or is not a directory." << "\n";
			return false;
		}
		if(exists(destination))
		{
			std::cerr << "Destination directory " << destination.string() << " already exists." << "\n";
			return false;
		}
		// create the destination directories
		if(!create_directories(destination))
		{
			std::cerr << "Unable to create destination directory" << destination.string() << "\n";
			return false;
		}
	}
	catch(filesystem_error const & e)
	{
		std::cerr << e.what() << "\n";
		return false;
	}

	// iterate through the source directory
	for(directory_iterator file(source); file != directory_iterator(); ++file)
	{
		try
		{
			path current(file->path());
			if(is_directory(current))
			{
				// Found directory: Recursion
				if(!copy_directory_recursive(current, destination / current.filename()))
					return false;
			}
			else
			{
				// Found file: Copy
				copy_file(current,destination / current.filename());
			}
		}
		catch(filesystem_error const & e)
		{
			std::cerr << e.what() << "\n";
		}
	}
	return true;
}
}


bool
urdf_to_osim(const std::string& path_to_urdf, const std::string& dir_osim_output)
{
	// ----------------------------------------------------------------------------------------------

	// > directories

	auto dir_urdf               = boost::filesystem::path(path_to_urdf).parent_path().string();
	auto dir_mesh_visual_src    = dir_urdf + "/../meshes/visual";
	auto dir_mesh_visual_dst    = dir_osim_output + "/meshes/visual";
	auto dir_mesh_collision_src = dir_urdf + "/../meshes/collision";
	auto dir_mesh_collision_dst = dir_osim_output + "/meshes/collision";

	// check that source files and directories exist
	// if so, create necessary destination directories and copy files

	if (not boost::filesystem::exists(path_to_urdf))
	{
		ROS_ERROR("URDF file could not be located, expected: '%s'.", path_to_urdf.c_str());
		return false;
	}

	if (not boost::filesystem::exists(dir_mesh_visual_src))
	{
		ROS_ERROR("Could not locate visual meshes directory, expected: '%s'.", dir_mesh_visual_src.c_str());
		return false;
	}
	else
	{
		ROS_INFO("Copying visual meshes.");
		boost::filesystem::copy_directory_recursive(dir_mesh_visual_src, dir_mesh_visual_dst);
	}
	

	if (not boost::filesystem::exists(dir_mesh_collision_src))
	{
		ROS_ERROR("Could not locate collision meshes directory, expected: '%s'.", dir_mesh_collision_src.c_str());
		return false;
	}
	else
	{
		ROS_INFO("Copying collision meshes.");
		boost::filesystem::copy_directory_recursive(dir_mesh_collision_src, dir_mesh_collision_dst);
	}

	// info
	
	ROS_INFO("URDF filepath: '%s'.", path_to_urdf.c_str());
	ROS_INFO("OSIM output directory: '%s'.", dir_osim_output.c_str());
	std::cout << "\n";

	// ----------------------------------------------------------------------------------------------

	// > load URDF model

	urdf::Model urdf_model;
	if (not urdf_model.initFile(path_to_urdf)){
		ROS_ERROR("Failed to parse URDF file.");
		return false;
	}

	ROS_INFO("Loaded URDF model.");

	// ----------------------------------------------------------------------------------------------

	// > OSIM model creation
	// convert KDL tree to OSIM model

	using namespace SimTK;

	OpenSim::Model osim_model;
	osim_model.setAuthors("Autogenerated using urdf_to_osim");
	osim_model.setName(urdf_model.getName());

	// create ground/world link

	bool fix_orientation = true;
	auto vec_ground_ori = (fix_orientation) ? Vec3(-M_PI/2, 0, 0) : Vec3(0);

	// iterate links of URDF model, constructing necessary OSIM model elements
	for (const auto& [link_name, link] : urdf_model.links_)
	{
		// skip world link
		if (link_name == "world")
			continue;

		// variables necesasy to define OSIM model
		auto link_nr        = get_link_number(link_name);
		auto link_mass      = INFINITY;
		auto link_cog       = Vec3(0);
		auto link_inertia   = Inertia(0);
		auto link_pos       = Vec3(0);
		auto link_ori       = Vec3(0);

		// inertial properties
		if (const auto I = link->inertial)
		{	
			link_mass    = I->mass;
			link_cog     = Vec3(I->origin.position.x, I->origin.position.y, I->origin.position.z);
			link_inertia = Inertia(I->ixx, I->iyy, I->izz, I->ixy, I->ixz, I->iyz);
		}

		// construct OSIM elements

		// names
		auto body_name      = "link" + std::to_string(link_nr);
		auto joint_name     = "joint" + std::to_string(link_nr - 1);
		auto actuator_name  = joint_name + "_actuator";

		// auto body_prev = osim_model.getGround();

		// body/link
		auto body  = new OpenSim::Body(body_name, link_mass, link_cog, link_inertia);

		// joint connecting to previous body/link
		auto joint = new OpenSim::PinJoint
		(
			joint_name,                                       // joint name
			osim_model.getGround(), link_pos, link_ori,                   // parent body, location in parent, orientation in parent
			*body, Vec3(0, 0, 0), Vec3(0, 0, 0) // child body, location in child, orientation in child
		);

		// display geometry
		auto mesh_visual = new OpenSim::Mesh("meshes/visual/" + body_name + ".stl");
		body->attachGeometry(mesh_visual->clone());

		// actuators
		auto actuator = new OpenSim::CoordinateActuator(joint->getCoordinate().getName());
		actuator->setName(actuator_name);
		actuator->setOptimalForce(100.0);

		// add components to model
		osim_model.addBody(body);
		osim_model.addJoint(joint);
		osim_model.addForce(actuator);
		
		// log information
		constexpr auto COLW = 8;
		std::cout << std::left
				  << "> " << link_name << "\n\n"
				  << std::setw(COLW) << "nr: "     << link_nr << "\n"
				  << std::setw(COLW) << "body: "   << body_name << "\n"
				  << std::setw(COLW) << "joint: "  << joint_name << "\n"
				  << std::setw(COLW) << "mass: "   << link_mass << " [kg]" << "\n"
				  << std::setw(COLW) << "CoG:\n"   << std::endl << link_cog << "\n\n"
				  << std::setw(COLW) << "inertia:" << std::endl << link_inertia << "\n"
				  << std::string(40, '-') << "\n\n"
				  << std::flush;
	}
	// ----------------------------------------------------------------------------------------------

	// finalize model
	osim_model.finalizeConnections();

	// controller (optional)
	;

	// export
	auto path_model = dir_osim_output + "/" + urdf_model.getName() + ".osim";
	ROS_INFO("Exporting model to: '%s'", path_model.c_str());
	osim_model.print(path_model);

	return true;
}

void
inertia_test()
{
	// What is the problem of having an inertia tensor not satisfying the triangle inequality?
	// https://physics.stackexchange.com/questions/348944/what-is-the-problem-of-having-an-inertia-tensor-not-satisfying-the-triangle-ineq
	
	// parallel axis theorem for tensor:
	// https://lh3.googleusercontent.com/proxy/A7nXJUYUgxJMo_3Dc15sPmA_UkKlhcD4ByzHqX_d4ySkSUfyQQtra3NGYG9nX8T-3l1TdQcb6uDWBrq3XPogqKelx4BjIrI

	// estimated dynamic parameters
	// https://github.com/marcocognetti/FrankaEmikaPandaDynModel
	// https://github.com/marcocognetti/FrankaEmikaPandaDynModel/blob/master/pdf/RA-L_2019_PandaDynIdent_SUPPLEMENTARY_MATERIAL.pdf
	// https://hal.inria.fr/hal-02265293/document

	std::cout << "inertia test\n\n";

	auto m_2 = 0.646926;
	auto CF_2 = SimTK::Vec3(-3.141e-03, -2.872e-02, 3.495e-03);
	auto I_2 = SimTK::Inertia(7.9620e-03, 2.8110e-02, 2.5995e-02, -3.9250e-03, 1.0254e-02, 7.0400e-04);
	std::cout << "I_2:\n" << I_2 << std::endl;

	std::cout << "shifting to center of mass..." << std::endl;
	I_2.shiftToMassCenter(CF_2, m_2);
	std::cout << "I_2':\n" << I_2 << std::endl;
	
	auto m_3 = 3.228604;
	auto CF_3 = SimTK::Vec3(2.7518e-02, 3.9252e-02, -6.6502e-02);
	auto I_3 = SimTK::Inertia(3.7242e-02, 3.6155e-02, 1.0830e-02, -4.7610e-03, -1.1396e-02, -1.2805e-02);
	std::cout << "I_3:\n" << I_3 << std::endl;

	std::cout << "shifting to center of mass..." << std::endl;
	I_3.shiftToMassCenter(CF_3, m_3);
	std::cout << "I_3':\n" << I_3 << std::endl;

	std::cout << "done!" << std::endl;
}

// ----------------------------------------------------------------------------------------------

int
main(int argc, char** argv)
{
	// verfify that correct number of arguments passed
	if (argc != 2 && argc !=3)
	{
		std::cerr << "Usage:   urdf_to_osim input_file [absolute_output_dir]" << std::endl;
		std::cerr << "Example: urdf_to_osim panda.urdf ~/path/to/output/dir" << std::endl;
		return 0;
	}

	// get parameters
	auto path_to_urdf    = boost::filesystem::current_path().string() + "/" + argv[1];
	auto dir_osim_output = argv[2] ? std::string(argv[2]) : boost::filesystem::current_path().string() + "/osim-model";
	
	// expand output directory
	wordexp_t exp_result;
	wordexp(dir_osim_output.c_str(), &exp_result, 0);
	dir_osim_output = exp_result.we_wordv[0];

	// create OSIM model
	auto osim_model = urdf_to_osim(path_to_urdf, dir_osim_output);

	// export OSIM model to specified path
	;

	return 0;
}