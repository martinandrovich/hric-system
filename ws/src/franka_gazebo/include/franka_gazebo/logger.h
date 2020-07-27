#pragma once

#include <string>
#include <fstream>

#define DEFAULT_DIR_UNIX "/home/panda2/Desktop/hric-system/assets/data"

template<typename Tq, typename Tqdot, typename Ttau_des, typename Tg>
static inline void
log_panda_info(const std::string& filename, const double time, const Tq& q, const Tqdot& qdot, const Ttau_des& tau_des, const Tg& g, const std::string& dir = DEFAULT_DIR_UNIX)
{
	
	static std::ofstream file;
	const std::string path = dir + "/" + filename + ".csv";

	if (not file.is_open())
	{
		// first line, open in overwrite mode and add header
		file.open(path, std::ios_base::out);
		file << "t, q0, q1, q2, q3, q4, q5, q6, qdot0, qdot1, qdot2, qdot3, qdot4, qdot5, qdot6, tau_des0, tau_des1, tau_des2, tau_des3, tau_des4, tau_des5, tau_des6, g0, g1, g2, g3, g4, g5, g6" << "\n";
		file.close();

		// re-open file in append mode
		file.open(path, std::ios_base::app);
	}
	
	// lambda for appending vector data
	static const auto append_vec = [&](const auto& vec) {
		for (size_t i = 0; i < 6; ++i)
			file << vec[i] << ", ";
		file << vec[6];
	};

	// time
	file << time;

	// current joint position
	file << ", ";
	append_vec(q);

	// current joint velocity
	file << ", ";
	append_vec(qdot);

	// desired joint torque
	file << ", ";
	append_vec(tau_des);

	// gravity vector
	file << ", ";
	append_vec(g);
	
	// end line
	file << "\n";
}