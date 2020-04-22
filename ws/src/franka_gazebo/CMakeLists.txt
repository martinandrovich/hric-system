cmake_minimum_required(VERSION 2.8.3)
project(franka_gazebo)

# C++17
set(CMAKE_CXX_STANDARD 17)

# dependencies (catkin packages)
find_package(
	catkin REQUIRED
	COMPONENTS
	roscpp
	xacro
	tf
	controller_manager
	effort_controllers
	joint_state_publisher
	robot_state_publisher
	joint_state_controller
	joint_trajectory_controller
	gazebo_ros
	gazebo_ros_control
	orocos_kdl
	kdl_parser
)

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
	LIBRARIES dynamics_lib
	CATKIN_DEPENDS controller_manager
	DEPENDS EIGEN3
)

# include dirs
include_directories(
	include
	${catkin_INCLUDE_DIRS}
	SYSTEM ${EIGEN3_INCLUDE_DIRS}
)

# libraries

## hw_node library
# https://roboticsbackend.com/ros-include-cpp-header-from-another-package/
add_library(dynamics_lib src/dynamics_lib.cpp)
target_link_libraries(dynamics_lib ${catkin_LIBRARIES})

install(
	TARGETS dynamics_lib
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

## install all project namespaced headers (package/include)
install(
	DIRECTORY include/${PROJECT_NAME}/
	DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)