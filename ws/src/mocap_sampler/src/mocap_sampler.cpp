#include <mocap_sampler/mocap_sampler.h>

bool
mocap_sampler::connect(const std::string& ip_addr)
{
	
	// print version info
	unsigned char ver[4];
	NatNet_GetVersion(ver);
	ROS_INFO("NatNet Sample Client (NatNet ver. %d.%d.%d.%d)", ver[0], ver[1], ver[2], ver[3]);

	// create NatNet client
	if (client != nullptr) delete client;
	client = new NatNetClient();

	// configure connection parameters
	connect_params.connectionType = ConnectionType_Multicast;
	connect_params.serverAddress = ip_addr.c_str();

	// release previous server
	is_connected = false;
	client->Disconnect();

	// init client and connect to NatNet server
	if (auto ret_code = client->Connect(connect_params); ret_code != ErrorCode_OK)
	{
		ROS_ERROR("Unable to connect to server (error code: %d)", ret_code);
		return false;
	}
	else
	{
		// connection succeeded
		// retrieve and print information about the connection

		void *pResult;
		int nBytes = 0;
		ErrorCode ret = ErrorCode_OK;

		// get server info
		memset(&server_desc, 0, sizeof(server_desc));
		ret = client->GetServerDescription(&server_desc);
		if (ret != ErrorCode_OK || !server_desc.HostPresent)
		{
			ROS_ERROR("Unable to connect to server, host not present");
			return false;
		}

		printf("\n\n[SampleClient] Server application info:\n");
		printf("Application: %s (ver. %d.%d.%d.%d)\n", server_desc.szHostApp, server_desc.HostAppVersion[0],
			   server_desc.HostAppVersion[1], server_desc.HostAppVersion[2], server_desc.HostAppVersion[3]);
		printf("NatNet Version: %d.%d.%d.%d\n", server_desc.NatNetVersion[0], server_desc.NatNetVersion[1],
			   server_desc.NatNetVersion[2], server_desc.NatNetVersion[3]);
		printf("Client IP:%s\n", connect_params.localAddress);
		printf("Server IP:%s\n", connect_params.serverAddress);
		printf("Server Name:%s\n", server_desc.szHostComputerName);

		// get mocap frame rate
		ret = client->SendMessageAndWait("FrameRate", &pResult, &nBytes);
		if (ret == ErrorCode_OK)
		{
			float fRate = *((float *)pResult);
			ROS_INFO("MOCAP framerate : %3.2f\n", fRate);
		}

		// get # of analog samples per mocap frame of data
		ret = client->SendMessageAndWait("AnalogSamplesPerMocapFrame", &pResult, &nBytes);
		if (ret == ErrorCode_OK)
		{
			analog_samples_per_mocap_frame = *((int*)pResult);
			printf("Analog Samples Per Mocap Frame : %d\n", analog_samples_per_mocap_frame);
		}

	}

	// everything went well
	ROS_INFO_STREAM("Connection to client on \"" << ip_addr << "\" has been established");
	is_connected = true;
	return true;
}

void
mocap_sampler::configure_sampling()
{
	// check if connection to client is established
	if (not is_connected)
	{
		ROS_ERROR("Cannot configure sampling without a connection to client; first run connect(ip_addr)");
		return;
	}

	// configure NatNet data handler callback
	client->SetFrameReceivedCallback(data_handler_callback, client);

	// configure publisher(s)
	pub_marker_pos = std::make_unique<ros::Publisher>(nh->advertise<geometry_msgs::Point>("/" + node_name + "/marker", 1));

	// finalize configuration
	is_configued = true;
}

ErrorCode
mocap_sampler::send_command(const std::string& cmd, void* response)
{
	// NatNet commands
	// https://v22.wiki.optitrack.com/index.php?title=NatNet:_Remote_Requests/Commands
	
	// check if connection to client is established
	if (not is_connected)
	{
		ROS_ERROR("Cannot send message without a connection to client; first run connect(ip_addr)");
		return ErrorCode::ErrorCode_InvalidOperation;
	}
	
	void *response_temp;
	int num_bytes;
	auto res = client->SendMessageAndWait(cmd.c_str(), &response_temp, &num_bytes);

	if (res != ErrorCode_OK)
		ROS_WARN("Error sending '%s' command", cmd.c_str());

	return res;
}

void NATNET_CALLCONV
mocap_sampler::data_handler_callback(sFrameOfMocapData* data, void* p_user)
{
	// check if system is configureds
	if (not is_configued)
	{
		ROS_ERROR("Data handler callback cannot run when sampling is unconfigured; first run configure_sampling()");
		return;
	}

	// handle incoming data (markers, rigid bodies, skeletons etc.)
	
	ROS_WARN_STREAM_ONCE("num markers: " << data->nLabeledMarkers);
	ROS_WARN_STREAM_ONCE("rigid bodies: " << data->nRigidBodies);


	// position of rigid body

	for (size_t i = 0; i < data->nRigidBodies; ++i)
	{
		static geometry_msgs::Point marker;
		const auto rigid_body = data->RigidBodies[i];

		marker.x = rigid_body.x;
		marker.y = rigid_body.y;
		marker.z = rigid_body.z;

		if (pub_marker_pos)
			pub_marker_pos->publish(marker);

		std::cout << "marker position:\n"
		          << "x: " << marker.x << "\n"
		          << "y: " << marker.y << "\n"
		          << "z: " << marker.z << "\n"
		          << std::endl;
	}
}

int
main(int argc, char* argv[])
{
	// initialize node with handle
	ros::init(argc, argv, mocap_sampler::node_name);
	mocap_sampler::nh = std::make_unique<ros::NodeHandle>(ros::NodeHandle());

	// check correct input
	if (argc != 2)
	{
		ROS_ERROR("IP address was not specified");
		return -1;
	}

	// get IP address from input
	const auto ip_addr = std::string(argv[1]);

	// connect to MOCAP system
	mocap_sampler::connect(ip_addr);

	// configure sampling
	mocap_sampler::configure_sampling();

	// async callbacks
	// ros::AsyncSpinner spinner(0);
	// spinner.start();

	// begin sampling
	mocap_sampler::send_command("LiveMode");

	// wait for user input

	// spin ROS
	while(ros::ok())
	{
		static auto i = 0;
		i++;
	}

	// stop sampling

	
	return 0;
}
