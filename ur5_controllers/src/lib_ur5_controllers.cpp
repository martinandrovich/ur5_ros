#include "ur5_controllers/ur5_controllers.h"

#include <ros/ros.h>
#include <ros_utils/ros.h>

void
ur5::command_setpoint(const Eigen::Vector6d& q_d, bool block)
{
	static ur5_msgs::RobotState state;
	for (auto i = 0; i < ur5::NUM_JOINTS; i++)
		state.q[i] = q_d[i];

	ros::topic::publish(ur5::COMMAND_TOPIC_JNT_POS, state, true);

	if (block) // change to feedback on (q - q_d) > 0
		ros::Duration(1.5).sleep();
}

void
ur5::command_setpoint(const trajectory_msgs::JointTrajectoryPoint& setpoint)
{
	// convert to ur5_msgs::RobotState (for now)
	static ur5_msgs::RobotState state;
	std::copy(setpoint.positions.begin(), setpoint.positions.end(), state.q.begin());
	std::copy(setpoint.velocities.begin(), setpoint.velocities.end(), state.qd.begin());
	std::copy(setpoint.accelerations.begin(), setpoint.accelerations.end(), state.qdd.begin());
	
	ros::topic::publish(ur5::COMMAND_TOPIC_JNT_POS, state, true);
}

void
ur5::command_traj(const trajectory_msgs::JointTrajectory& traj)
{
	// assuming linear dt for all trajectory points
	const auto dt = std::abs(traj.points[0].time_from_start.toSec() - traj.points[1].time_from_start.toSec());
	const auto freq = 1/dt;

	ROS_INFO_STREAM("Commanding UR5 joint trajectory at " << freq << " [Hz]...");

	// iterate and command points
	ros::Rate lp(freq);
	for (const auto& setpoint : traj.points)
	{
		// command setpoint and sleep for dt
		ur5::command_setpoint(setpoint);
		lp.sleep();
	}
}