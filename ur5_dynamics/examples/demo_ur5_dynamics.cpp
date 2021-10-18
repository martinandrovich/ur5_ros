#include <iostream>

#include <ros/ros.h>
#include <ur5_dynamics/ur5_dynamics.h>

int main(int argc, char** argv)
{
	ros::init(argc, argv, "demo_ur5_dynamics");
	ros::NodeHandle nh;

	// first run:
	// roslaunch ur5_gazebo ur5.launch ee:=wsg50

	// kinematics

	const auto q       = (Eigen::Vector6d() << 1.57, -1.57, 1.57, 1.57, 1.57, 0).finished();
	const auto b_T_ee  = ur5_dynamics::fwd_kin(q);
	const auto pose_ee = ur5_dynamics::fwd_kin<geometry_msgs::Pose>(q);
	const auto vec_q   = ur5_dynamics::inv_kin(b_T_ee);
	const auto q_best  = ur5_dynamics::inv_kin(b_T_ee, q); // best q wrt. Euclidean distance
	
	const auto J       = ur5_dynamics::jac(q);
	const auto J_pinv  = ur5_dynamics::pinv_jac(J);
	
	std::cout.precision(3);
	std::cout << "q:\n"          << q << "\n\n";
	std::cout << "b_T_ee:\n"     << b_T_ee.matrix() << "\n\n";
	std::cout << "pose_ee:\n"    << pose_ee << "\n";
	std::cout << "vec_q[0]:\n"   << vec_q[0] << "\n\n";
	std::cout << "q_best:\n"     << q_best << "\n\n";
	
	std::cout << "J:\n"          << J << "\n\n";
	std::cout << "det(J):\n"     << J.determinant() << "\n\n";
	std::cout << "inv(J):\n"     << J.inverse() << "\n\n";
	std::cout << "J_pinv:\n"     << J_pinv << "\n\n";
	std::cout << "J * inv(J):\n" << J * J.inverse() << "\n\n";
	std::cout << "J * J_pinv:\n" << J * J_pinv << "\n\n";

	// dynamics

	// ...

	return 0;
}
