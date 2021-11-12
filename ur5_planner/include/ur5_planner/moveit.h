#pragma once

#include <string>
#include <mutex>
#include <unordered_map>

#include <ros/ros.h>
#include <moveit/planning_interface/planning_interface.h>

#include <geometry_msgs/Pose.h>
#include <trajectory_msgs/JointTrajectory.h>

enum class Planner { RRT, RRTConnect, RRTstar };

namespace ur5::moveit
{

	// -- variables + constants -----------------------------------------------------
	
	static inline std::unordered_map<Planner, std::string> PLANNERS = // cannot be const since [] operator is non-const
	{
		{ Planner::RRT,        "RRT" },
		{ Planner::RRTConnect, "RRTConnect" },
		{ Planner::RRTstar,    "RRTstar" }
	};
	
	static inline std::vector<std::string> gazebo_cobj_exclude = { "ur5", "ground_plane", "camera_stereo", "openni_kinect", "projector" };
	
	static inline const std::string PLANNER_PLUGIN             = "ompl_interface/OMPLPlanner";
	static inline const std::string ARM_GROUP                  = "ur5_arm";
	static inline const std::string EE_GROUP                   = "ur5_ee";
	static inline const std::string PLANNING_SCENE_TOPIC       = "/planning_scene_gazebo";
	
	// -- methods -------------------------------------------------------------------
	
	bool
	init(ros::NodeHandle& nh);
	
	void
	terminate();

	void
	update_planning_scene_from_gazebo(bool remove_attached_cobjs = true);
	
	void
	publish_planning_scene();
	
	void
	print_planning_scene();
	
	void
	start_scene_publisher(double freq = 50);

	void
	attach_object_to_ee(const std::string& name);
	
	void
	move_base(const geometry_msgs::Pose& pose);

	// -- planning ------------------------------------------------------------------
	
	using Plan = planning_interface::MotionPlanResponse;
	
	Plan
	plan(
		const geometry_msgs::Pose& pose_des,
		const Planner& planner,
		double max_planning_time = 1.0, // [s]
		size_t max_planning_attempts = 10
	);

	trajectory_msgs::JointTrajectory
	plan_to_jnt_traj(
		Plan&  plan,
		double dt            = 0.001,  // s
		double tol           = 0.0001, // rad
		double max_vel_scale = 1.0,    // rad/s
		double max_acc_scale = 1.0     // rad/s^2
	);
	
	template<typename T>
	void
	set_planner_config(const Planner& planner, const std::string& property, const T& value)
	{
		if (auto param = "/planner_configs/" + PLANNERS[planner] + "/" + property; not ros::param::has(param))
			throw std::runtime_error("The parameter '" + param + "' does not exist in set_planner_config().");
		else
			ros::param::set(param, value);
	}
}