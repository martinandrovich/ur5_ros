#pragma once

#include <string>
#include <vector>
#include <set>
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
	static inline const auto JOINT_STATE_TOPIC  = ros::param::read<std::string>("JOINT_STATE_TOPIC", "/joint_states");
	static inline const auto JNT_NAMES          = ros::param::read<std::vector<std::string>>("/ur5_joint_position_controller/joint_names", {});

	static inline const auto l6_T_ee            = Eigen::Translation3d(0, 0.0823, 0) * Eigen::Isometry3d::Identity();
	static inline const auto ee_T_tcp           = []() {
		const auto v = ros::param::read<std::vector<double>>("ee_T_tcp", { 0, 0, 0 });
		return Eigen::Translation3d(v[0], v[1], v[2]) * Eigen::Isometry3d::Identity();
	}();

	static inline const auto has_ee             = []() { return not ur5::ee_T_tcp.matrix().isIdentity(); };

	static inline const class
	{
		// usage
		// LINKS[0], LINKS["base"], LINKS["b"] --> "ur5::link0"
		// LINKS.URDF("base") --> "link0"

	private:

		std::vector<std::pair<std::set<std::string>, std::string>> v =
		{
			{ {"link0", "b" , "base", "first" }, ur5::ROBOT_NAME + "::link0" },
			{ {"link1", "l1",                 }, ur5::ROBOT_NAME + "::link1" },
			{ {"link2", "l2",                 }, ur5::ROBOT_NAME + "::link2" },
			{ {"link3", "l3",                 }, ur5::ROBOT_NAME + "::link3" },
			{ {"link4", "l4",                 }, ur5::ROBOT_NAME + "::link4" },
			{ {"link5", "l5",                 }, ur5::ROBOT_NAME + "::link5" },
			{ {"link6", "l6", "last"          }, ur5::ROBOT_NAME + "::link6" },
			{ {"ee", "flange", "end-effector" }, ur5::ROBOT_NAME + "::ee"    },
		};

	public:

		auto
		operator [](size_t i) const { return v[i].second; }

		auto
		operator [](const std::string& s) const
		{
			const auto it = std::find_if(v.begin(), v.end(), [&](const auto& p){
				return (p.first.find(s) != p.first.end());
			});
			return v[std::distance(v.begin(), it)].second;
		}

		template<typename T>
		auto
		URDF(const T& i) const
		{
			const auto s = this->operator[](i);
			return s.substr(s.find("::") + 2);
		}

	} LINKS;

	static inline const auto ROBOT_DESCRIPTION2 = []() { return ros::param::read<std::string>("ROBOT_DESCRIPTION2", "/default"); };
}