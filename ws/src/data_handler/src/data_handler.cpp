#include "data_handler.h"

int
main(int argc, char** argv)
{
	
	// initialize data handler node
	data_handler::init(argc, argv);

	// async callbacks
	ros::AsyncSpinner spinner(DATA_HANDLER_NUM_THREADS);
	spinner.start();

	// sclear screen after init
	ros::Duration(DATA_HANDLER_LOAD_TIMEOUT).sleep();
	system("clear");

	// sync loop
	std::thread t_sync([&]()
	{
		ros::Rate sync_freq(10);
		while (ros::ok())
		{
			data_handler::sync();
			sync_freq.sleep();
		}
	});

	// gui loop
	std::thread t_gui([&]()
	{
		ros::Rate gui_freq(DATA_HANDLER_GUI_FREQ);
		while (ros::ok())
		{
			data_handler::print_gui();
			gui_freq.sleep();
		}
	});

	// wait for threads and shutdown
	t_sync.join();
	t_gui.join();	
	ros::waitForShutdown();

	return 0;
}