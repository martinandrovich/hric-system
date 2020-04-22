#include <franka_sim_controllers/joint_position_pd_gravity_controller.h>
#include <pluginlib/class_list_macros.hpp>

// export controller
PLUGINLIB_EXPORT_CLASS(franka_sim_controllers::JointPositionPDGravityController, controller_interface::ControllerBase)