#pragma once

#include <string>
#include <Eigen/Eigen>

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Pose.h>

#include <ur5_description/ur5.h>

namespace ur5_gazebo
{

	// -- transformations -----------------------------------------------------------
	
	Eigen::Isometry3d
	w_T_b();
	
	// -- state and pose ------------------------------------------------------------
	
	sensor_msgs::JointState
	get_robot_state();

	sensor_msgs::JointState
	get_gripper_state();
	
	geometry_msgs::Pose
	// get_pose("l6", "base");
	get_pose(const std::string& link, const std::string& reference_frame);

	geometry_msgs::Pose
	get_base_pose();
	
	// get_pose(link, reference_frame)
	// get_tf("w_T_l6");
	// get_tf("w", "l6");
	// get_pose("w_T_l6");
	// get_pose("l6", "world");
	
	geometry_msgs::Pose
	get_link6_pose(bool in_world_frame = false);

	geometry_msgs::Pose
	get_ee_pose(bool in_world_frame = false);

	geometry_msgs::Pose
	get_tcp_pose(bool in_world_frame = false);
	
	geometry_msgs::Pose
	get_ee_given_pos(const geometry_msgs::Pose& pose_obj, const Eigen::Isometry3d& offset = Eigen::Translation3d(0.0, 0.0, 0.0) * Eigen::Isometry3d::Identity());
	
	geometry_msgs::Pose
	get_tcp_given_pos(const geometry_msgs::Pose& pose_obj, const Eigen::Isometry3d& offset = Eigen::Translation3d(0.0, 0.0, 0.0) * Eigen::Isometry3d::Identity());

}