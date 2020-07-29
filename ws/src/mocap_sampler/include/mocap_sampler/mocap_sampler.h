#pragma once

#include <string>
#include <memory>

#include <ros/ros.h>
#include <geometry_msgs/Point.h>

#include <NatNet/NatNetTypes.h>
#include <NatNet/NatNetCAPI.h>
#include <NatNet/NatNetClient.h>

int
ConnectClient();

namespace mocap_sampler
{

	bool
	connect(const std::string& ip_addr);

	void
	configure_sampling();

	ErrorCode
	send_command(const std::string& cmd, void* response = nullptr);

	void NATNET_CALLCONV
	data_handler_callback(sFrameOfMocapData* data, void* p_user);

	std::string node_name = "mocap_sampler";
	bool is_configued = false;
	bool is_connected = false;
	std::unique_ptr<ros::NodeHandle> nh;
	std::unique_ptr<ros::Publisher> pub_marker_pos;

	NatNetClient* client = nullptr;
	sNatNetClientConnectParams connect_params;
	sServerDescription server_desc;
	int analog_samples_per_mocap_frame = 0;

}