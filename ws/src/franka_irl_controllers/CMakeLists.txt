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

## ...
add_library(franka_irl_controllers src/cartesian_position_controller.cpp)
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