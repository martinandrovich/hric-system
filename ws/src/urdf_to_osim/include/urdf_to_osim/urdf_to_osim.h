#pragma once

#include <string>

bool
urdf_to_osim(const std::string& path_to_urdf, const std::string& dir_osim_output);

void
export_osim();
