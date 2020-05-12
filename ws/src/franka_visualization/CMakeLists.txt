cmake_minimum_required(VERSION 3.4)
project(franka_visualization)

# C++17
set(CMAKE_CXX_STANDARD 17)

# dependencies (catkin packages)
find_package(
	catkin REQUIRED
	COMPONENTS
	roscpp
	sensor_msgs
	franka_description
)

# declare a catkin package
catkin_package(
	CATKIN_DEPENDS xacro
)