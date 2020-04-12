#include <franka_gazebo/dynamics.h>

bool
franka_gazebo::dynamics::init()
{
	// should only be initialized once
	if (is_sys_init)
		return true;

	// load URDF robot model
	if (not robot_model.initParam(ROBOT_DESCRIPTION))
	{
		ROS_ERROR("Could not load URDF robot model from '%s'.", ROBOT_DESCRIPTION.c_str());
		return false;
	}

	// compose KDL tree
	KDL::Tree kdl_tree;
	if (not kdl_parser::treeFromUrdfModel(robot_model, kdl_tree))
	{
		ROS_ERROR("Could not construct KDL tree from robot model.");
		return false;
	};

	// load KDL chain
	// kdl_tree.getChain("panda_link0", "panda_link8", kdl_chain);
	kdl_tree.getChain(BASE_LINK, LAST_LINK, kdl_chain);

	for (size_t n : { 0, 1, 2, 3, 4, 5, 6, 7 })
		ROS_INFO_STREAM("segment : " << kdl_chain.getSegment(n).getName());

	// initialize KDL solver(s)
	kdl_dyn_solver = new KDL::ChainDynParam(kdl_chain, KDL::Vector(0, 0, -GRAVITY));

	// initialize joint arrays
	q     = KDL::JntArray(NUM_JOINTS);
	qdot  = KDL::JntArray(NUM_JOINTS);
	qddot = KDL::JntArray(NUM_JOINTS);
	G     = KDL::JntArray(NUM_JOINTS);

	// done
	is_sys_init = true;
	ROS_INFO("[OK] Initialized Franka Panda dynamics library.");
}

void
franka_gazebo::dynamics::callback_joint_state(const sensor_msgs::JointStateConstPtr& msg)
{
	if (not is_sys_init)
		return;

	// copy joint state
	mtx_joint_state.lock();
	joint_state = *msg;
	mtx_joint_state.unlock();

	// callback has been called at least once; joint state arrays are now intialized
	is_joint_state_init = true;

	// invoke computation of dynamics
	dynamics::compute();
}

bool
franka_gazebo::dynamics::init_check()
{
	if (not is_sys_init)
	{
		ROS_ERROR("System has yet not been initialized.");
		return false;
	}

	if (not is_joint_state_init)
	{
		ROS_ERROR("Joint state has yet not been initialized.");
		return false;
	}

	return true;
}

void
franka_gazebo::dynamics::compute()
{
	if (not init_check())
		return;

	// acquire mutex
	std::lock_guard lock(mtx_joint_state);

	// load values of q and qdot from joint_state into joint array(s)
	for (size_t i = 0; i < NUM_JOINTS; ++i)
	{
		q(i)    = joint_state.position[i];
		qdot(i) = joint_state.velocity[i];
	}

	// compute gravity
	kdl_dyn_solver->JntToGravity(q, G);
}

KDL::JntArray
franka_gazebo::dynamics::position()
{
	if (not init_check())
		ROS_WARN("Returning undefined vector.");

	std::lock_guard lock(mtx_joint_state);
	return q;
}

KDL::JntArray
franka_gazebo::dynamics::velocity()
{
	if (not init_check())
		ROS_WARN("Returning undefined vector.");

	std::lock_guard lock(mtx_joint_state);
	return qdot;
}

KDL::JntArray
franka_gazebo::dynamics::gravity()
{
	if (not init_check())
		ROS_WARN("Returning undefined vector.");

	std::lock_guard lock(mtx_joint_state);
	return G;
}