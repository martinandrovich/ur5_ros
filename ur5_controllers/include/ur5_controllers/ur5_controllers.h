#pragma once

#include <vector>
#include <string>
#include <Eigen/Eigen>

#include <ur5_description/ur5_description.h>
#include <ur5_msgs/PoseTwist.h>
#include <ur5_msgs/RobotState.h>
#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

namespace ur5
{
	static inline constexpr auto    EXEC_FREQ             = 1000.0; // [Hz]
	static inline constexpr auto    EXEC_DT               = 1./EXEC_FREQ; // [s]
	static inline const std::string	COMMAND_TOPIC_JNT_POS = "/ur5_joint_position_controller/command";
	// static inline const std::string	COMMAND_TOPIC_CAR_POS = "/ur5_joint_position_controller/command";

	void // make templated? .. remove?
	command_setpoint(const Eigen::Vector6d& q_d, bool block = true);
	
	void
	command_setpoint(const trajectory_msgs::JointTrajectoryPoint& setpoint);

	void
	command_traj(const trajectory_msgs::JointTrajectory& traj);
}