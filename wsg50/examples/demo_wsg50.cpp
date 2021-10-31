#include <ros/ros.h>
#include <wsg50/wsg50.h>

int main(int argc, char** argv)
{
	ros::init(argc, argv, "demo_wsg50");
	ros::NodeHandle nh;
	
	// grasp while blocking
	wsg50::grasp(true);
	wsg50::release(true);
	wsg50::grasp(true);
	wsg50::release(true);

	return 0;
}
