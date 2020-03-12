#pragma once

#include <stdexcept>
#include <string>

#include <ros/ros.h>

// generic error messages

constexpr auto    ERR_ROS_NO_PARAM            = "Could not locate parameter: %s";
constexpr auto    ERR_ROS_HELPER_NOT_INIT     = "No nodes are initialized; helper class cannot execute.";

// helper methods

template<typename T>
inline auto
getParam(std::string param, const std::string& ns = "/hric_sys/")
{
	param = ns + param;

	if (not ros::isInitialized())
		ROS_ERROR(ERR_ROS_HELPER_NOT_INIT);

	if (not ros::param::has(param))
		ROS_ERROR("Could not locate parameter: %s", param.c_str());

	T val;
	ros::param::get(param, val);

	return val;
}