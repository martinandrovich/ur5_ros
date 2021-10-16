#include <string>

#include <ros/ros.h>
#include <ur5_gazebo/ur5_gazebo.h>

int main(int argc, char** argv)
{
	ros::init(argc, argv, "demo_ur5_gazebo");
	ros::NodeHandle nh;

	// first run
	// ...

	// transforms

	auto w_T_b  = ur5_gazebo::w_T_b();
	auto b_T_l6 = ur5_gazebo::get_tf("b", "l6");
	auto w_T_ee = ur5_gazebo::get_tf("w", "ee");
	auto b_T_ee = ur5_gazebo::get_tf("b", "ee");

	std::cout << "w_T_b:\n"  << w_T_b.matrix()  << std::endl;
	std::cout << "b_T_l6:\n" << b_T_l6.matrix() << std::endl;
	std::cout << "w_T_ee:\n" << w_T_ee.matrix() << std::endl;
	std::cout << "b_T_ee:\n" << b_T_ee.matrix() << std::endl;

	// poses

	auto pose_l6 = ur5_gazebo::get_pose("l6");
	auto pose_ee = ur5_gazebo::get_pose("ee");

	std::cout << "pose_l6:\n" << pose_l6 << std::endl;
	std::cout << "pose_ee:\n" << pose_ee << std::endl;

	return 0;
}
