#pragma once

#include <string>
#include <mutex>

#include <ros/ros.h>
#include <data_handler/data_handler.h>
#include <data_handler/GenericData.h>
#include <data_handler/RegisterHWNode.h>
#include <std_msgs/UInt32.h>

struct HWNodeStats
{
	using clk = std::chrono::high_resolution_clock;
	using tp = std::chrono::time_point<clk>;

	tp time_prev;
	size_t num_trans = 0;
	double freq = 0.0;
	double freq_mean = 0.0;   // mean
	double freq_sqd = 0.0;    // square distance from mean
	double freq_var = 0.0;    // variance
	double freq_svar = 0.0;   // sample variance
	double freq_stddev = 0.0; // std deviation

	void
	reset()
	{
		num_trans = 0;
		freq = 0.0;
		freq_mean = 0.0;
		freq_sqd = 0.0;
		freq_var = 0.0;
		freq_svar = 0.0;
		freq_stddev = 0.0;
	}

	void
	update()
	{
		// frequency and number of transmissions
		freq = (1.0 / (clk::now() - time_prev).count()) * 1e9;
		time_prev = clk::now();
		num_trans++;

		// running mean and variance
		// https://en.wikipedia.org/w/index.php?title=Algorithms_for_calculating_variance#Welford's_online_algorithm

		if (num_trans < 2)
			return;

		auto delta1 = freq - freq_mean;
		freq_mean += delta1 / (double)num_trans;
		auto delta2 = freq - freq_mean;
		freq_sqd += delta1 * delta2;

		freq_var = freq_sqd / num_trans;
		freq_svar = freq_sqd / double(num_trans - 1);
		freq_stddev = std::sqrt(freq_var);
	}
};

struct HWNode
{
	std::string name;
	std::string topic;
	size_t hz;
	bool use_dynamic_freq;
	ros::Subscriber sub;
	ros::Publisher pub;
	data_handler::GenericData msg;
	HWNodeStats stats;
};

class hw_node
{
public:

	template<typename T>
	static bool
	init(ros::NodeHandle& nh, size_t hz, const std::string& name = ros::this_node::getName())
	{
		// set variables
		hw_node::nh                  = &nh;
		hw_node::name                = name;
		hw_node::topic               = hw_node::name.substr(name.find_last_of('/') + 1);
		hw_node::topic_full          = DATA_HANDLER_TOPIC_NS_IN + hw_node::topic;
		hw_node::hz                  = hz;
		hw_node::use_dynamic_freq    = false;

		// advertise topic (e.g /data_handler/in/topic_name)
		hw_node::pub                 = hw_node::nh->advertise<data_handler::GenericData>(hw_node::topic_full, DATA_HANDLER_QUEUE_SIZE);

		// register at data_handler
		hw_node::client              = hw_node::nh->serviceClient<data_handler::RegisterHWNode>(DATA_HANDLER_SRV_REGHWN);

		data_handler::RegisterHWNode srv;

		srv.request.name             = hw_node::name;
		srv.request.topic            = hw_node::topic;
		srv.request.hz               = hw_node::hz;
		srv.request.use_dynamic_freq = hw_node::use_dynamic_freq;

		if (not client.call(srv) || not srv.response.success)
		{
			ROS_ERROR("Failed to register '%s' with data handler.", name.c_str());
			return false;
		}

		// done
		hw_node::is_init = true;
		ROS_INFO("Initialized hw_node: '%s' [%lu Hz][%s]", hw_node::name.c_str(), hw_node::hz, hw_node::topic_full.c_str());
		return true;
	}

	template<typename T>
	static void
	publish(const T& data)
	{
		// check initialization
		if (not hw_node::is_init)
		{
			ROS_ERROR("Cannot publish from HW node (%s); is not initialized.", ros::this_node::getName().c_str());
			return;
		}

		// create message
		static data_handler::GenericData msg;
		msg.header.stamp    = ros::Time::now();
		msg.header.frame_id = "";

		// serialize input data
		msg.size = ros::serialization::serializationLength(data);
		msg.data.resize(msg.size);
		ros::serialization::OStream ostream(msg.data.data(), msg.size);
		ros::serialization::serialize(ostream, data);

		// publish message
		hw_node::pub.publish(msg);

		// info
		ROS_INFO("Published data to data_handler from '%s'.", hw_node::name.c_str());
	}

	template<typename T>
	static void
	subscribe(ros::NodeHandle* nh, const std::string& topic, std::function<void(const T*)> callback)
	{
		// generic subscriber callback
		auto callback_impl = new boost::function<void(const data_handler::GenericDataConstPtr&)>([=](const auto& msg) -> void
		{			
			// notify data_handler
			;
			
			// convert data
			static T msgout;
			ros::serialization::IStream istream(const_cast<uint8_t*>(msg->data.data()), msg->size);
			ros::serialization::deserialize(istream, msgout);

			// callback
			callback(&msgout);
		});
		
		// subsribe to ROS topic
		auto topic_full = DATA_HANDLER_TOPIC_NS_OUT + topic;
		hw_node::sub = nh->subscribe(topic_full, DATA_HANDLER_QUEUE_SIZE, *callback_impl, ros::VoidConstPtr(), ros::TransportHints().tcpNoDelay().udp());

		// info
		ROS_INFO("%s subsribed to %s.", ros::this_node::getName().c_str(), topic.c_str());
	}

private:

	static inline bool is_init               = false;
	static inline ros::NodeHandle* nh        = nullptr;
	
	static inline std::string name           = "";
	static inline std::string topic          = "";
	static inline std::string topic_full     = "";
	static inline size_t hz                  = 0;
	static inline bool use_dynamic_freq      = false;

	static inline ros::Publisher pub;
	static inline ros::Subscriber sub;
	static inline ros::ServiceClient client;
};

