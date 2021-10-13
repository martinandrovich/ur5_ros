#include <string>

#include <ros/ros.h>
#include <ros_utils/ros.h>
#include <ur5_description/ur5.h>

int main(int argc, char** argv)
{
	ros::init(argc, argv, "demo_ur5_params");
	ros::NodeHandle nh;
	
	// retrieve all parameters
	std::cout << "testing parameters from ur5_description..\n\n";
	
	std::cout << "ur5::GRAVITY:           " << ur5::GRAVITY << std::endl;
	std::cout << "ur5::NUM_JOINTS:        " << ur5::NUM_JOINTS << std::endl;
	std::cout << "ur5::ROBOT_NAME:        " << ur5::ROBOT_NAME << std::endl;
	std::cout << "ur5::ROBOT_DESCRIPTION: " << ur5::ROBOT_DESCRIPTION << std::endl;
	std::cout << "\n";
	
	// test dynamic parameter loading
	ros::param::del("ROBOT_DESCRIPTION2");
	std::cout << "ur5::ROBOT_DESCRIPTION2: " << ur5::ROBOT_DESCRIPTION2() << std::endl;
	std::cout << "setting '/ROBOT_DESCRIPTION2' to '/monkey'..." << std::endl;
	ros::param::set("ROBOT_DESCRIPTION2", "/monkey");
	std::cout << "ur5::ROBOT_DESCRIPTION2: " << ur5::ROBOT_DESCRIPTION2() << std::endl;
	std::cout << "\n";
	
	std::cin.get();
	
	return 0;
}
