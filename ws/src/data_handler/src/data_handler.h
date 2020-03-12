# pragma once

#include <chrono>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>
#include <thread>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <ros/transport_hints.h>

#include <data_handler/GenericData.h>
#include <data_handler/RegisterHWNode.h>
#include <data_handler/TestMessage.h>
#include <data_handler/data_handler.h>
#include <data_handler/hw_node.h>
#include <data_handler/utils.h>

#include "ansi.h"

// declarations

namespace data_handler
{
	using clk = std::chrono::high_resolution_clock;
	using tp = std::chrono::time_point<clk>;

	bool is_init = false;
	ros::NodeHandle* nh = nullptr;
	ros::ServiceServer server;

	std::mutex mutex_hw_nodes;
	std::unordered_map<std::string, HWNode*> hw_nodes;

	void
	init(int argc, char** argv);

	bool
	hw_node_service(data_handler::RegisterHWNode::Request& req, data_handler::RegisterHWNode::Response& res);

	bool
	register_hw_node(const std::string& name, const std::string& topic, size_t hz, bool dynamic_frequency = false);

	void
	update_hw_node(const std::string& topic, const data_handler::GenericDataConstPtr& msg);

	void
	sync();

	void
	print_gui();
}

// implementation

void
data_handler::init(int argc, char** argv)
{
	// assert that system is not already initialized
	if (data_handler::is_init)
	{
		ROS_ERROR("Data handler has already been initialized once.");
		return;
	}

	// check that parameters have been initialized
	;

	// initialize node with handle
	ros::init(argc, argv, "data_handler");
	data_handler::nh = new ros::NodeHandle;

	// log configuration status
	data_handler::is_init = true;
	ROS_INFO("The '%s' node has been configured.", ros::this_node::getName().c_str());
	// ROS_INFO_STREAM("data_handler main thread id: " << boost::this_thread::get_id());

	// advertise service for hw nodes
	data_handler::server = data_handler::nh->advertiseService(DATA_HANDLER_SRV_REGHWN, data_handler::hw_node_service);
	ROS_INFO("Waiting to register HW nodes...\n");
}

bool
data_handler::hw_node_service(data_handler::RegisterHWNode::Request& req, data_handler::RegisterHWNode::Response& res)
{
	res.success = register_hw_node(req.name, req.topic, req.hz, req.use_dynamic_freq) ? true : false;
	return true;
}

bool
data_handler::register_hw_node(const std::string& name, const std::string& topic, size_t hz, bool dynamic_frequency)
{
	// generic subscriber callback
	auto callback = new boost::function<void(const data_handler::GenericDataConstPtr&)>([=](const auto& msg) -> void
	{
		data_handler::update_hw_node(topic, msg);
	});

	// full input topic name
	auto topic_full = DATA_HANDLER_TOPIC_NS_IN + topic;

	// create HW node
	auto hw = new HWNode
	{
		name,
		topic,
		hz,
		false,
		data_handler::nh->subscribe(topic_full, DATA_HANDLER_QUEUE_SIZE, *callback, ros::VoidConstPtr(), ros::TransportHints().tcpNoDelay().udp()),
		data_handler::nh->advertise<data_handler::GenericData>(DATA_HANDLER_TOPIC_NS_OUT + topic, DATA_HANDLER_QUEUE_SIZE),
		data_handler::GenericData(),
		HWNodeStats()
	};

	// append node to map
	data_handler::mutex_hw_nodes.lock();
	data_handler::hw_nodes[topic] = hw;
	data_handler::mutex_hw_nodes.unlock();

	// info
	ROS_WARN("Registered: '%s' [%s][%lu hz]", topic.c_str(), topic_full.c_str(), hz);
	return true;
}

void
data_handler::update_hw_node(const std::string& topic, const data_handler::GenericDataConstPtr& msg)
{
	// mutex guard
	std::lock_guard l(mutex_hw_nodes);

	// get hw node
	const auto hwn = hw_nodes[topic];

	// update stored data
	hwn->msg = *(msg.get());

	// update stats
	hwn->stats.update();
}

void
data_handler::print_gui()
{

	// move cursor to origin
	std::cout << ANSI::CURSOR_HIDE << ANSI::CURSOR_MV_ORIGIN;

	// lock guard
	std::lock_guard l(mutex_hw_nodes);

	// print information (mutexed)
	std::cout << ANSI::FORMAT("DATA HANDLER STATUS:", ANSI::BLOCK_BLUE) << "\n\n";
	
	std::cout << "> number of HW nodes: " << hw_nodes.size() << "\n";
	std::cout << "> uptime: "             << "N/A" << "\n";

	std::cout << "\n" << ANSI::FORMAT("REGISTERED HW NODES:", ANSI::BLOCK_BLUE) << "\n";

	for (const auto& [topic, node] : hw_nodes)
		std::cout << "\n"
		          << std::setfill(' ')
		          << std::left << std::setw(20) << topic
		          << std::left << " | "
		          << std::left << "mean: " << std::right << std::setw(10) << node->stats.freq_mean << ""
		          << std::left << " | "
		          << std::left << "stddev: " << std::right << std::setw(10) << node->stats.freq_stddev << ""
		          << std::left << " | "
		          << std::left << "count: " << std::right << std::setw(10) << node->stats.num_trans << ""
		          << std::left << " | "
				  << "";

	// calculate longest string
	// auto MAX_LENGTH = std::max_element(map_stats.begin(), map_stats.end(), [](const auto& a, const auto& b) { return a.first.size() < b.first.size(); })->first.size();
}

void
data_handler::sync()
{
	// mutex guard
	std::lock_guard l(mutex_hw_nodes);

	// publish data from each node
	for (const auto& [topic, node] : hw_nodes)
		node->pub.publish(node->msg);
}