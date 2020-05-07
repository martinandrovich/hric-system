#pragma once

#include <string>

#include <ros/ros.h>
#include <kdl/kdl.hpp>
#include <kdl_parser/kdl_parser.hpp>

bool
urdf_to_osim(const std::string& path_to_urdf);

void
export_osim();
