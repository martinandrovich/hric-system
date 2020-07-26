cmake_minimum_required(VERSION 2.8.3)
project(franka_irl_controllers)

# C++17
set(CMAKE_CXX_STANDARD 17)

# Find catkin and any catkin packages
find_package(
	catkin REQUIRED
	COMPONENTS
	roscpp
	pluginlib
	controller_interface
	hardware_interface
	franka_hw
	geometry_msgs
	orocos_kdl
)

# find libfranka
find_package(Franka 0.7.0 REQUIRED)

# Finding Eigen is somewhat complicated because of our need to support Ubuntu
# all the way back to saucy.  First we look for the Eigen3 cmake module
# provided by the libeigen3-dev on newer Ubuntu.  If that fails, then we
# fall-back to the version provided by cmake_modules, which is a stand-in.
find_package(Eigen3 QUIET)
if(NOT EIGEN3_FOUND)
	find_package(cmake_modules REQUIRED)
	find_package(Eigen REQUIRED)
	set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
endif()

# Note that eigen 3.2 (on Ubuntu Wily) only provides EIGEN3_INCLUDE_DIR,
# not EIGEN3_INCLUDE_DIRS, so we have to set the latter from the former.
if(NOT EIGEN3_INCLUDE_DIRS)
	set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()

# declare a catkin package
catkin_package(
	INCLUDE_DIRS include
	LIBRARIES franka_irl_controllers
	CATKIN_DEPENDS
		roscpp
		pluginlib
		controller_interface
		hardware_interface
		franka_hw
		geometry_msgs
	DEPENDS
		Franka
		orocos_kdl
		EIGEN3
)

# include dirs
include_directories(
	include
	${catkin_INCLUDE_DIRS}
)

################################################################################################

# nodes

# ...

################################################################################################

# libraries

## controllers
add_library(franka_irl_controllers
	src/cartesian_position_controller.cpp
	src/joint_position_pd_gravity_controller.cpp
)
target_link_libraries(franka_irl_controllers ${catkin_LIBRARIES})

install(
	TARGETS franka_irl_controllers
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
	FILES franka_irl_controllers_plugin.xml
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)