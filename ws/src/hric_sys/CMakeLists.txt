cmake_minimum_required(VERSION 2.8.3)
project(hric_sys)

## C++17
add_compile_options(-std=c++17)

## Find catkin and any catkin packages
find_package(
	catkin
	REQUIRED COMPONENTS
	roscpp
	rospy
	std_msgs
	genmsg
	data_handler
)

## declare ROS messages and services
add_message_files(
	FILES
)

add_service_files(
	FILES
)

## generate added messages and services
generate_messages(
	DEPENDENCIES
	std_msgs
)

## declare a catkin package
catkin_package(
	INCLUDE_DIRS include
	CATKIN_DEPENDS message_runtime
	std_msgs
)

## include dirs
include_directories(include ${catkin_INCLUDE_DIRS})

## libraries
# https://answers.ros.org/question/201977/include-header-file-from-another-package-indigo/

### ...

## nodes

### data handler test app

#### talker node
add_executable(dh_talker src/apps/test/data_handler/dh_talker.cpp)
target_link_libraries(dh_talker ${catkin_LIBRARIES})
add_dependencies(dh_talker ${PROJECT_NAME}_generate_messages_cpp)

#### listener node
add_executable(dh_listener src/apps/test/data_handler/dh_listener.cpp)
target_link_libraries(dh_listener ${catkin_LIBRARIES})
add_dependencies(dh_listener ${PROJECT_NAME}_generate_messages_cpp)