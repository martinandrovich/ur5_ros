#include <string>

#include <ros/ros.h>
#include <ur5_gazebo/ur5_gazebo.h>

int main(int argc, char** argv)
{
	ros::init(argc, argv, "demo_ur5_gazebo");
	ros::NodeHandle nh;

	// first run
	// roslaunch ur5_gazebo ur5.launch ee:=wsg50

	// transforms
	auto w_T_b  = ur5::w_T_b();
	auto b_T_l6 = ur5::get_tf("b", "l6");
	auto w_T_ee = ur5::get_tf("w", "ee");
	auto b_T_ee = ur5::get_tf("b", "ee");

	std::cout << "w_T_b:\n"  << w_T_b.matrix()  << "\n";
	std::cout << "b_T_l6:\n" << b_T_l6.matrix() << "\n";
	std::cout << "w_T_ee:\n" << w_T_ee.matrix() << "\n";
	std::cout << "b_T_ee:\n" << b_T_ee.matrix() << "\n";

	// poses
	auto pose_l6 = ur5::get_pose("l6");
	auto pose_ee = ur5::get_pose("ee");
	auto pose_b  = ur5::get_pose("b", "world");

	std::cout << "pose_l6:\n" << pose_l6 << "\n";
	std::cout << "pose_ee:\n" << pose_ee << "\n";
	std::cout << "pose_b:\n"  << pose_b  << "\n";

	// states
	auto state    = ur5::get_state();
	auto state_ee = ur5::get_ee_state();
	
	std::cout << "state:\n" << state << "\n";
	std::cout << "state_ee:\n" << state_ee << "\n";

	return 0;
}
