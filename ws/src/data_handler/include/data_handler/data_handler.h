#pragma once

#include <data_handler/utils.h>

// constants

constexpr auto    DATA_HANDLER_NS                 = "/data_handler/";

#define           DATA_HANDLER_QUEUE_SIZE         getParam<int>("queue_size", DATA_HANDLER_NS)
#define           DATA_HANDLER_TOPIC_NS_IN        DATA_HANDLER_NS + getParam<std::string>("topic_ns_in", DATA_HANDLER_NS) + "/"
#define           DATA_HANDLER_TOPIC_NS_OUT       DATA_HANDLER_NS + getParam<std::string>("topic_ns_out", DATA_HANDLER_NS) + "/"
#define           DATA_HANDLER_SRV_REGHWN         DATA_HANDLER_NS + getParam<std::string>("service_register_hw_node", DATA_HANDLER_NS)
#define           DATA_HANDLER_SYNC_FREQ          getParam<int>("sync_freq", DATA_HANDLER_NS)
#define           DATA_HANDLER_GUI_FREQ           getParam<int>("gui_freq", DATA_HANDLER_NS)
#define           DATA_HANDLER_NUM_THREADS        getParam<int>("num_threads", DATA_HANDLER_NS)
#define           DATA_HANDLER_LOAD_TIMEOUT       getParam<int>("load_timeout", DATA_HANDLER_NS)