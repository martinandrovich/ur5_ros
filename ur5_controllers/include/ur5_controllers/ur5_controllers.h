#pragma once

#include <vector>
#include <string>
#include <Eigen/Eigen>

#include <ur5_description/ur5_description.h>
#include <ur5_msgs/PoseTwist.h>
#include <ur5_msgs/RobotState.h>
#include <sensor_msgs/JointState.h>

namespace ur5
{
	static inline constexpr auto    EXEC_FREQ             = 1000.0; // [Hz]
	static inline const std::string	COMMAND_TOPIC_JNT_POS = "/ur5_joint_position_controller/command";
	// static inline const std::string	COMMAND_TOPIC_CAR_POS = "/ur5_joint_position_controller/command";

	void // todo
	exec_traj(const std::vector<sensor_msgs::JointState>& traj, double freq = ur5::EXEC_FREQ);
	
	// template<typename T>
	void
	command(const Eigen::Vector6d& q_d, bool block = true);
}