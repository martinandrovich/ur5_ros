#pragma once

#include <string>
#include <Eigen/Eigen>

#include <ros/ros.h>
#include <ros_utils/ros.h>

namespace Eigen
{
	using Matrix6d = Eigen::Matrix<double, 6, 6>;
	using Vector6d = Eigen::Matrix<double, 6, 1>;
}

namespace ur5
{
	static inline const auto GRAVITY            = -9.80665;
	static inline const auto NUM_JOINTS         = ros::param::read<int>("NUM_JOINTS", 6);
	static inline const auto ROBOT_NAME         = ros::param::read<std::string>("ROBOT_NAME", "ur5");
	static inline const auto ROBOT_DESCRIPTION  = ros::param::read<std::string>("ROBOT_DESCRIPTION", "/robot_description");
	 
	static inline const auto ROBOT_DESCRIPTION2 = []() { return ros::param::read<std::string>("ROBOT_DESCRIPTION2", "/default"); };
}