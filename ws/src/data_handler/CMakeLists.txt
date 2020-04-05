cmake_minimum_required(VERSION 2.8.3)
project(data_handler)

# C++17
set(CMAKE_CXX_STANDARD 17)

# Find catkin and any catkin packages
find_package(
	catkin REQUIRED
	COMPONENTS
	roscpp
	rospy
	std_msgs
	genmsg
)

# declare ROS messages and services
add_message_files(
	FILES
	GenericData.msg
	TestMessage.msg
)

add_service_files(
	FILES
	RegisterHWNode.srv
)

# generate added messages and services
generate_messages(
	DEPENDENCIES
	std_msgs
)

# declare a catkin package
catkin_package(
	INCLUDE_DIRS include
	LIBRARIES hw_node_lib
	CATKIN_DEPENDS message_runtime std_msgs
)

# include dirs
include_directories(
	include
	${catkin_INCLUDE_DIRS}
)

# nodes

## data_handler node
add_executable(data_handler src/data_handler.cpp)
target_link_libraries(data_handler ${catkin_LIBRARIES})
add_dependencies(data_handler ${PROJECT_NAME}_generate_messages_cpp)

# libraries

## hw_node library
# https://roboticsbackend.com/ros-include-cpp-header-from-another-package/
add_library(hw_node_lib src/hw_node_lib.cpp)
target_link_libraries(hw_node_lib ${catkin_LIBRARIES})

install(
	TARGETS hw_node_lib
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

## install all project namespaced headers (package/include)
install(
	DIRECTORY include/${PROJECT_NAME}/
	DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)