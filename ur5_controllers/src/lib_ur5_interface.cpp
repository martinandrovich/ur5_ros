#include "ur5_controllers/interface.h"

#include <ros/ros.h>
#include <ros_utils/ros.h>

void
ur5::exec_traj(const std::vector<sensor_msgs::JointState>& traj, double freq)
{
	ROS_INFO_STREAM("Commanding UR5 joint trajectory at " << freq << " [Hz]...");
	
	ros::Rate lp(freq); // Hz
	for (const auto& msg : traj)
	{
		ros::topic::publish(ur5::COMMAND_TOPIC_JNT_POS, msg, true);
		lp.sleep();
	}
}