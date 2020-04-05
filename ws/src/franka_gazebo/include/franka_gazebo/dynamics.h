#pragma once

#include <string>
#include <mutex>
#include <optional>

#include <ros/ros.h>
#include <urdf/model.h>
#include <sensor_msgs/JointState.h>

#include <kdl_parser/kdl_parser.hpp>
#include <kdl/tree.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainjnttojacdotsolver.hpp>
#include <kdl/chainjnttojacsolver.hpp>

namespace franka_gazebo
{
class dynamics
{

public:

	static bool
	init();

	static void
	callback_joint_state(const sensor_msgs::JointStateConstPtr& msg);

	static std::optional<KDL::JntArray>
	get_gravity();

	static inline const std::string            ROBOT_NAME        = "panda";
	static inline const std::string            ROBOT_DESCRIPTION = "/robot_description";
	static inline constexpr auto               NUM_JOINTS        = 7;
	static inline const std::string            BASE_LINK         = ROBOT_NAME + "_link0";
	static inline const std::string            LAST_LINK         = ROBOT_NAME + "_link8";

private:

	static bool
	init_check();

	static void
	compute();

	static inline bool                         is_sys_init = false;
	static inline bool                         is_joint_state_init = false;

	static inline urdf::Model                  robot_model;

	static inline sensor_msgs::JointState      joint_state;
	static inline std::mutex                   mtx_joint_state;

	static inline KDL::JntArray                q;     // position
	static inline KDL::JntArray                qdot;  // velocity
	static inline KDL::JntArray                qddot; // acceleration
	static inline KDL::JntArray                G;     // gravity

	static inline KDL::Chain                   kdl_chain;
	static inline KDL::ChainDynParam*          kdl_dyn_solver;
	static inline KDL::ChainJntToJacSolver*    kdl_jac_solver;
	static inline KDL::ChainJntToJacDotSolver* kdl_jac_dot_solver;

};
}