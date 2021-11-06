#include "ur5_controllers/ur5_controllers.h"

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

void
ur5::command(const Eigen::Vector6d& q_d, bool block)
{
	static ur5_msgs::RobotState msg;
	
	for (auto i = 0; i < ur5::NUM_JOINTS; i++)
		msg.q[i] = q_d[i];
		
	ros::topic::publish(ur5::COMMAND_TOPIC_JNT_POS, msg, true);
	
	if (block) // change to feedback on (q - q_d) > 0
		ros::Duration(1.5).sleep();
}