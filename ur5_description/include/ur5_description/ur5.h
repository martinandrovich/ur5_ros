#pragma once

#include <string>
#include <vector>
#include <set>
#include <unordered_map>
#include <algorithm>
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
	static inline const auto JOINT_STATE_TOPIC  = ros::param::read<std::string>("JOINT_STATE_TOPIC", "/joint_state");

	static inline const auto ROBOT_DESCRIPTION2 = []() { return ros::param::read<std::string>("ROBOT_DESCRIPTION2", "/default"); };

	static inline const struct
	{
		// usage
		// LINKS[0] or LINKS["base"] or LINKS["b"] --> "ur5::ur5_link0"

		std::vector<std::pair<std::set<std::string>, std::string>> v =
		{
			{{"b",  "base"},  ur5::ROBOT_NAME + "::ur5_link0"},
			{{"l1", "link1"}, ur5::ROBOT_NAME + "::ur5_link1"},
			{{"l2", "link2"}, ur5::ROBOT_NAME + "::ur5_link2"},
			{{"l3", "link3"}, ur5::ROBOT_NAME + "::ur5_link3"},
			{{"l4", "link4"}, ur5::ROBOT_NAME + "::ur5_link4"},
			{{"l5", "link5"}, ur5::ROBOT_NAME + "::ur5_link5"},
			{{"l6", "link6"}, ur5::ROBOT_NAME + "::ur5_link6"},
			{{"ee"},          ur5::ROBOT_NAME + "::ur5_ee"},
			{{"tcp"},         ur5::ROBOT_NAME + "::ee_tcp"},
		};

		auto operator [](size_t i) const { return v[i].second; }

		auto operator [](const std::string& s) const
		{
			const auto it = std::find_if(v.begin(), v.end(), [&](const auto& p){
				return (p.first.find(s) != p.first.end());
			});
			return v[std::distance(v.begin(), it)].second;
		}

	} LINKS;
}