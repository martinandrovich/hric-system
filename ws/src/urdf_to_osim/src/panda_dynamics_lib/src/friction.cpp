#include "franka_dynamics/franka_dynamics.h"

Eigen::Vector7d
franka::dynamics::friction(const Eigen::Vector7d& qdot)
{

    using namespace std;

	static Eigen::Vector7d tau_f = Eigen::Vector7d::Zero();

	tau_f(0) =  FI_11/(1+exp(-FI_21*(qdot(0)+FI_31))) - TAU_F_CONST_1;
	tau_f(1) =  FI_12/(1+exp(-FI_22*(qdot(1)+FI_32))) - TAU_F_CONST_2;
	tau_f(2) =  FI_13/(1+exp(-FI_23*(qdot(2)+FI_33))) - TAU_F_CONST_3;
	tau_f(3) =  FI_14/(1+exp(-FI_24*(qdot(3)+FI_34))) - TAU_F_CONST_4;
	tau_f(4) =  FI_15/(1+exp(-FI_25*(qdot(4)+FI_35))) - TAU_F_CONST_5;
	tau_f(5) =  FI_16/(1+exp(-FI_26*(qdot(5)+FI_36))) - TAU_F_CONST_6;
	tau_f(6) =  FI_17/(1+exp(-FI_27*(qdot(6)+FI_37))) - TAU_F_CONST_7;

	return tau_f;
}
