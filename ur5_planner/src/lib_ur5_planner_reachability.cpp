#include "ros/duration.h"
#include "ros_utils/std.h"
#include "ur5_planner/reachability.h"
#include "ur5_planner/moveit.h"

#include <tuple>
#include <Eigen/Eigen>

#include <ros/ros.h>
#include <ros_utils/ros_utils.h>
#include <geometry_msgs/Pose.h>
#include <ur5_dynamics/ur5_dynamics.h>


ur5::moveit::ReachabilityData
ur5::moveit::reachability(const geometry_msgs::Pose& pose_base, const geometry_msgs::Pose& pose_obj, const Eigen::Isometry3d& T_grasp, size_t resolution, bool visualize)
{
	// the object at pose_obj (w_T_obj) is used to compute a desired EE pose (w_T_ee = w_T_obj) given a grasp transformation T_grasp
	// the reachability test tries to grasp the object by rotating around its z-axis given a resolution, such that
	// b_T_ee = b_T_w * w_T_obj * rot_z(angle) * T_grasp
	
	// initilaize data
	ReachabilityData data = { geometry_msgs::read_pose(pose_base).pos, resolution, 0, 0, 0 };

	// move robot base
	ur5::moveit::move_base(pose_base);

	// define transformations
	auto b_T_w   = Eigen::make_tf(pose_base).inverse();
	auto w_T_obj = Eigen::make_tf(pose_obj);

	// rotate around object's z-axis given the resolution and grasp transformation
	for (auto [i, theta] = std::tuple{0, 0.}; i <= resolution and ros::ok(); i++)
	{
		// update orientation
		theta = 2.* i * (M_PI/resolution);

		// define transformations
		auto R_z = Eigen::AngleAxisd(theta, Eigen::Vector3d::UnitZ());
		auto b_T_ee = b_T_w * w_T_obj * R_z * T_grasp;

		// calculate the inverse_kinematics to the pose
		auto vec_q = ur5_dynamics::inv_kin(b_T_ee);

		// iterate possible joint configurations (solutions)
		for (const auto& q : vec_q)
		{
			if (visualize) // sleep to allow publisher to visualize
				ros::Duration(0.5).sleep();

			// get robot state (for this scope)
			auto& robot_state = ur5::moveit::get_mutexed_planning_scene()->getCurrentStateNonConst();

			// set joint states
			robot_state.setJointGroupPositions(ARM_GROUP, q);
			robot_state.setJointGroupPositions(EE_GROUP, std::vector{0.05, 0.05});
			
			// check for collisions and update reachability data
			data.states++;
			data.collisions += check_collision() ? 1 : 0;
			data.plausible_states = data.states - data.collisions;
		}
	}
	
	// return reachability data	
	return data;
}