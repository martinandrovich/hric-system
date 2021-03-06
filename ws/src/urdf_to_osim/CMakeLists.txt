cmake_minimum_required(VERSION 2.8.3)
project(urdf_to_osim)

# C++17
set(CMAKE_CXX_STANDARD 17)

# find catkin and any catkin packages
find_package(
	catkin REQUIRED
	COMPONENTS
	roscpp
	roslib
	xacro
	urdf
)

# find franka_dynamics lib (custom)
# set(franka_dynamics_INCLUDE_DIR include)
# set(franka_dynamics_LIBRARIES ${CMAKE_SOURCE_DIR}/lib/franka_dynamics/franka_dynamics.a)

# find OpenSim + SimBody
# list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
set(OpenSim_DIR "~/opensim/lib/cmake/OpenSim")
find_package(OpenSim REQUIRED)

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
	CATKIN_DEPENDS roscpp roslib
	DEPENDS EIGEN3 OpenSim
)

# include dirs
include_directories(
	include
	${catkin_INCLUDE_DIRS}
	${OpenSim_INCLUDE_DIRS}
	${EIGEN3_INCLUDE_DIRS}
)

################################################################################################

# libraries

## lib_urdf_to_osim
add_library(lib_urdf_to_osim src/urdf_to_osim.cpp)
target_link_libraries(lib_urdf_to_osim ${catkin_LIBRARIES} ${OpenSim_LIBRARIES})

install(
	TARGETS lib_urdf_to_osim
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

# lib_pd_controller
set(franka_dynamics_LIBRARIES ${PROJECT_SOURCE_DIR}/lib/franka_dynamics.a)
add_library(lib_pd_controller src/pd_controller/pd_controller.cpp)
target_link_libraries(lib_pd_controller ${catkin_LIBRARIES} ${OpenSim_LIBRARIES} ${franka_dynamics_LIBRARIES})

## install all project namespaced headers (package/include)
install(
	DIRECTORY include/${PROJECT_NAME}/
	DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

################################################################################################

# nodes

## urdf_to_osim executable (rosrun)
add_executable(urdf_to_osim src/urdf_to_osim.cpp)
target_link_libraries(urdf_to_osim ${catkin_LIBRARIES} ${OpenSim_LIBRARIES} lib_urdf_to_osim lib_pd_controller)

## franka_panda_example
add_executable(franka_panda_example src/franka_panda_example.cpp)
target_link_libraries(franka_panda_example ${catkin_LIBRARIES} ${OpenSim_LIBRARIES} lib_pd_controller)