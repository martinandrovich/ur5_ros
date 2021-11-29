#include <string>

#include <ros/ros.h>
#include <ros_utils/ros.h>
#include <ur5_description/ur5_description.h>

int main(int argc, char** argv)
{
	ros::init(argc, argv, "demo_ur5_description");
	ros::NodeHandle nh;
	
	// first run:
	// roslaunch ur5_gazebo ur5.launch ee:=wsg50
	
	// LINKS[] data structure
	std::cout << "ur5::LINKS[0]: "             << ur5::LINKS[0] << std::endl;
	std::cout << "ur5::LINKS[\"b\"]: "         << ur5::LINKS["b"] << std::endl;
	std::cout << "ur5::LINKS[\"base\"]: "      << ur5::LINKS["base"] << std::endl;
	std::cout << "ur5::LINKS[\"last\"]: "      << ur5::LINKS["last"] << std::endl;
	std::cout << "ur5::LINKS.URDF(\"last\") "  << ur5::LINKS.URDF("last") << std::endl;
	std::cout << "\n";

	// parameters
	// loaded from parameter server, or set to default value
	std::cout << "ur5::GRAVITY: "              << ur5::GRAVITY << std::endl;
	std::cout << "ur5::NUM_JOINTS: "           << ur5::NUM_JOINTS << std::endl;
	std::cout << "ur5::ROBOT_NAME: "           << ur5::ROBOT_NAME << std::endl;
	std::cout << "ur5::JOINT_STATE_TOPIC: "    << ur5::JOINT_STATE_TOPIC << std::endl;
	std::cout << "\n";
	
	// transformations and EE
	// 'ee_T_tcp' is defined on the ROS parameter server!
	std::cout << "ur5::l6_T_ee:\n"             << ur5::l6_T_ee.matrix() << std::endl;
	std::cout << "ur5::ee_T_tcp:\n"            << ur5::ee_T_tcp().matrix() << std::endl; // loaded dynamically!
	std::cout << "ur5::has_ee(): "             << std::boolalpha << ur5::has_ee() << std::endl;
	std::cout << "\n";
	
	return 0;
}
