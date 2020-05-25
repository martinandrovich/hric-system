#pragma once

/*
 *  Created on: 24 feb 2019
 *  Authors: Oliva Alexander, Gaz Claudio, Cognetti Marco
 * 
 *  Modified on: 21 may 2020
 *  Modified by: Martin Androvich
 *
 *  C. Gaz, M. Cognetti, A. Oliva, P. Robuffo Giordano, A. De Luca, 'Dynamic
 *  Identification of the Franka Emika Panda Robot With Retrieval of Feasible
 *  Parameters Using Penalty-Based Optimization'. IEEE RA-L, 2019.
 */

#include <cmath>
#include <Eigen/Core>

namespace Eigen
{
	using Matrix7d = Eigen::Matrix<double, 7, 7>;
	using Vector7d = Eigen::Matrix<double, 7, 1>;
}

namespace franka::dynamics
{
	using namespace std;

	// constant precomputations

	constexpr auto FI_11 = 0.54615;
	constexpr auto FI_12 = 0.87224;
	constexpr auto FI_13 = 0.64068;
	constexpr auto FI_14 = 1.2794;
	constexpr auto FI_15 = 0.83904;
	constexpr auto FI_16 = 0.30301;
	constexpr auto FI_17 = 0.56489;

	constexpr auto FI_21 = 5.1181;
	constexpr auto FI_22 = 9.0657;
	constexpr auto FI_23 = 10.136;
	constexpr auto FI_24 = 5.5903;
	constexpr auto FI_25 = 8.3469;
	constexpr auto FI_26 = 17.133;
	constexpr auto FI_27 = 10.336;

	constexpr auto FI_31 = 0.039533;
	constexpr auto FI_32 = 0.025882;
	constexpr auto FI_33 = -0.04607;
	constexpr auto FI_34 = 0.036194;
	constexpr auto FI_35 = 0.026226;
	constexpr auto FI_36 = -0.021047;
	constexpr auto FI_37 = 0.0035526;

	const auto TAU_F_CONST_1 = FI_11 / (1 + exp(-FI_21 * FI_31));
	const auto TAU_F_CONST_2 = FI_12 / (1 + exp(-FI_22 * FI_32));
	const auto TAU_F_CONST_3 = FI_13 / (1 + exp(-FI_23 * FI_33));
	const auto TAU_F_CONST_4 = FI_14 / (1 + exp(-FI_24 * FI_34));
	const auto TAU_F_CONST_5 = FI_15 / (1 + exp(-FI_25 * FI_35));
	const auto TAU_F_CONST_6 = FI_16 / (1 + exp(-FI_26 * FI_36));
	const auto TAU_F_CONST_7 = FI_17 / (1 + exp(-FI_27 * FI_37));

	// methods
	// https://simtk-confluence.stanford.edu:8443/display/OpenSim/How+Inverse+Dynamics+Works

	Eigen::Matrix7d
	mass(const Eigen::Vector7d& q);

	// Eigen::Matrix7d
	// coriolis(const Eigen::Vector7d& q, const Eigen::Vector7d& qdot);

	Eigen::Vector7d
	gravity(const Eigen::Vector7d& q);

	Eigen::Vector7d
	friction(const Eigen::Vector7d& qdot);

}
