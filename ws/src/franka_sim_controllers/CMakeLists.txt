cmake_minimum_required(VERSION 2.8.3)
project(franka_sim_controllers)

# C++17
set(CMAKE_CXX_STANDARD 17)

# Find catkin and any catkin packages
find_package(
	catkin REQUIRED
	COMPONENTS
	roscpp
	franka_gazebo
	pluginlib
)

# declare a catkin package
catkin_package(
	INCLUDE_DIRS include
	LIBRARIES franka_sim_controllers
	CATKIN_DEPENDS franka_gazebo
)

# include dirs
include_directories(
	include
	${catkin_INCLUDE_DIRS}
)

# find franka_dynamics lib (custom)
set(franka_dynamics_LIBRARIES ${PROJECT_SOURCE_DIR}/lib/franka_dynamics.a)

################################################################################################

# nodes

## gravity_control node
add_executable(gravity_control src/gravity_control.cpp)
target_link_libraries(gravity_control ${catkin_LIBRARIES})

## joint_trajectory_control node
add_executable(joint_trajectory_control src/joint_trajectory_control.cpp)
target_link_libraries(joint_trajectory_control ${catkin_LIBRARIES})

################################################################################################

# libraries

## ...
add_library(franka_sim_controllers src/joint_position_pd_gravity_controller.cpp)
target_link_libraries(franka_sim_controllers ${catkin_LIBRARIES} ${franka_dynamics_LIBRARIES})

install(
	TARGETS franka_sim_controllers
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

## install all project namespaced headers (package/include)
install(
	DIRECTORY include/${PROJECT_NAME}/
	DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

## install ros_control plugin
install(
	FILES franka_sim_controllers_plugin.xml
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)