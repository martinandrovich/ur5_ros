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
#include <kdl/trajectory_composite.hpp>

namespace ur5
{
	static inline constexpr auto    EXEC_FREQ             = 1000.0; // [Hz]
	static inline constexpr auto    EXEC_DT               = 1./EXEC_FREQ; // [s]
	static inline const std::string	COMMAND_TOPIC_JNT_POS = "/ur5_joint_position_controller/command";
	// static inline const std::string	COMMAND_TOPIC_CAR_POS = "/ur5_joint_position_controller/command";
	
	void // make templated?
	command_setpoint(const Eigen::Vector6d& q_d, bool block = true);
	
	void
	command_setpoint(const trajectory_msgs::JointTrajectoryPoint& setpoint);

	void
	command_traj(const trajectory_msgs::JointTrajectory& traj);
	
	void
	command_traj(const std::shared_ptr<KDL::Trajectory_Composite>& traj, double dt = 0.001);
	
	inline void
	command_home()
	{
		ROS_INFO_STREAM("Commanding UR5 home...");
		command_setpoint((Eigen::Vector6d() << 1.57, -1.57, 1.57, 1.57, 1.57, 0).finished(), true);
	}
}