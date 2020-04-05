cmake_minimum_required(VERSION 2.8.3)
project(franka_gazebo)

## C++17
set(CMAKE_CXX_STANDARD 17)

## dependencies (catkin packages)
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
)

# declare a catkin package
catkin_package(
	CATKIN_DEPENDS controller_manager
)

## include dirs
include_directories(${catkin_INCLUDE_DIRS})

## add custom controller as library
# add_library(${PROJECT_NAME}_controllers_lib src/joint_trajectory_controller.cpp)
# target_link_libraries(${PROJECT_NAME}_controllers_lib ${catkin_LIBRARIES})