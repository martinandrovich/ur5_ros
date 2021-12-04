#include <wsg50/wsg50.h>

#include <ros/ros.h>
#include <ros_utils/ros.h>

namespace wsg50
{
	static inline std_msgs::Float64 msg_tau_des;
}

void
wsg50::grasp(bool block)
{
	msg_tau_des.data = EFFORT_GRASP;
	ros::topic::publish(COMMAND_TOPIC, msg_tau_des, true);
	ROS_INFO_STREAM("Setting gripper to: " << msg_tau_des.data << " [Nm]...");
	
	if (block)
		ros::Duration(2.0).sleep();
}

void
wsg50::release(bool block)
{
	msg_tau_des.data = EFFORT_RELEASE;
	ros::topic::publish(COMMAND_TOPIC, msg_tau_des, true);
	ROS_INFO_STREAM("Setting gripper to: " << msg_tau_des.data << " [Nm]...");
	
	if (block)
		ros::Duration(2.0).sleep();
}
