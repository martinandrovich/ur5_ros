#pragma once

#include <Eigen/Eigen>
#include <ros_utils/ros_utils.h>
#include <ur5_description/ur5_description.h>
#include <geometry_msgs/Pose.h>

const auto VISUALIZE_REACHABILITY = true;
const auto GRASP_SIDE_AT_TCP = [](auto z = 0.1){ return ur5::ee_T_tcp.inverse() * Eigen::make_tf({0, 0, z}); };
// const auto GRASP_TOP_AT_TCP  = [](auto z = 0.1){ return ur5::ee_T_tcp.inverse() * Eigen::make_tf({0, 0, z}); };

namespace ur5::moveit
{	

	struct
	ReachabilityData
	{
		std::array<double, 3> pos_base;
		size_t resolution, states, collisions, plausible_states;
	};

	ReachabilityData
	reachability(
		const geometry_msgs::Pose& pose_base,
		const geometry_msgs::Pose& pose_obj,
		const Eigen::Isometry3d& T_grasp,
		size_t resolution = 16, 
		bool visualize = false
	);
	
	void // todo
	export_reachability_data(const std::string& filename);
}