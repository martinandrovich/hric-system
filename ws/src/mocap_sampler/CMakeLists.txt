cmake_minimum_required(VERSION 2.8.3)
project(mocap_sampler)

# C++17
set(CMAKE_CXX_STANDARD 17)

# find catkin and any catkin packages
find_package(
	catkin REQUIRED
	COMPONENTS
	roscpp
	roslib
	geometry_msgs
)

## find NatNet SDK (custom)
set(NatNet_INCLUDE_DIR include)
set(NatNet_LIBRARIES ${PROJECT_SOURCE_DIR}/lib/libNatNetLibShared.so)

# find franka_dynamics lib (custom)
# set(franka_dynamics_INCLUDE_DIR include)
# set(franka_dynamics_LIBRARIES ${CMAKE_SOURCE_DIR}/lib/franka_dynamics/franka_dynamics.a)

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
	LIBRARIES lib_urdf_to_osim
	CATKIN_DEPENDS
		roscpp
		roslib
		geometry_msgs
	DEPENDS
		EIGEN3
		NatNet
)

# include dirs
include_directories(
	include
	${catkin_INCLUDE_DIRS}
	${OpenSim_INCLUDE_DIRS}
	${EIGEN3_INCLUDE_DIRS}
)

################################################################################################

# nodes

## urdf_to_osim executable (rosrun)
add_executable(mocap_sampler src/mocap_sampler.cpp)
target_link_libraries(mocap_sampler ${catkin_LIBRARIES} ${OpenSim_LIBRARIES} ${NatNet_LIBRARIES})