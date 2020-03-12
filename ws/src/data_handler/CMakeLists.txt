cmake_minimum_required(VERSION 2.8.3)
project(data_handler)

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
)

## declare ROS messages and services
add_message_files(
	FILES
	GenericData.msg
	TestMessage.msg
)

add_service_files(
	FILES
	RegisterHWNode.srv
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

## nodes

### data_handler node
add_executable(data_handler src/data_handler.cpp)
target_link_libraries(data_handler ${catkin_LIBRARIES})
add_dependencies(data_handler ${PROJECT_NAME}_generate_messages_cpp)

### data_handler lib
# ...

### hw_node library
add_library(hw_node_lib src/hw_node.cpp)
target_link_libraries(hw_node_lib ${catkin_LIBRARIES})
add_dependencies(hw_node_lib ${PROJECT_NAME}_generate_messages_cpp)

### test talker node
add_executable(hwtalker src/hwtalker.cpp)
target_link_libraries(hwtalker ${catkin_LIBRARIES} hw_node_lib)
add_dependencies(hwtalker ${PROJECT_NAME}_generate_messages_cpp)

### test listener node
add_executable(hwlistener src/hwlistener.cpp)
target_link_libraries(hwlistener ${catkin_LIBRARIES} hw_node_lib)
add_dependencies(hwlistener ${PROJECT_NAME}_generate_messages_cpp)

## install all project namespaced headers (package/include)
install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION} FILES_MATCHING PATTERN "*.h" PATTERN ".svn" EXCLUDE)