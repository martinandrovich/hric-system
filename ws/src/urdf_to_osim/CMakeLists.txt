cmake_minimum_required(VERSION 2.8.3)
project(urdf_to_osim)

# C++17
set(CMAKE_CXX_STANDARD 17)

# find catkin and any catkin packages
find_package(
	catkin REQUIRED
	COMPONENTS
	roscpp
	xacro
	orocos_kdl
	kdl_parser
)

# find OpenSim + SimBody
# list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
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

## ...
add_library(lib_urdf_to_osim src/urdf_to_osim.cpp)
target_link_libraries(lib_urdf_to_osim ${catkin_LIBRARIES} ${OpenSim_LIBRARIES})

install(
	TARGETS lib_urdf_to_osim
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

## install all project namespaced headers (package/include)
install(
	DIRECTORY include/${PROJECT_NAME}/
	DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

################################################################################################

# nodes

## urdf_to_osim executable (rosrun)
add_executable(urdf_to_osim src/urdf_to_osim.cpp)
target_link_libraries(urdf_to_osim ${catkin_LIBRARIES} lib_urdf_to_osim ${OpenSim_LIBRARIES})

## osim_example
add_executable(osim_example src/osim_example.cpp)
# target_include_directories(urdf_to_osim PRIVATE ${catkin_LIBRARIES} ${OpenSim_INCLUDE_DIRS})
target_link_libraries(osim_example ${catkin_LIBRARIES} ${OpenSim_LIBRARIES})