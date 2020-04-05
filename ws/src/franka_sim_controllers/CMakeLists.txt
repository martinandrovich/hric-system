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
)

# declare a catkin package
catkin_package(
	INCLUDE_DIRS
)

# include dirs
include_directories(${catkin_INCLUDE_DIRS})

################################################################################################

### sim_control_trajectory
add_executable(gravity_control src/gravity_control.cpp)
target_link_libraries(gravity_control ${catkin_LIBRARIES})

## sim_control_trajectory
add_executable(joint_trajectory_control src/joint_trajectory_control.cpp)
target_link_libraries(joint_trajectory_control ${catkin_LIBRARIES})