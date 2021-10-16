#pragma once

#include <string>
#include <Eigen/Eigen>

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Pose.h>

namespace ur5_gazebo
{
	
	// -- state and pose ------------------------------------------------------------
	
	sensor_msgs::JointState
	get_robot_state();

	sensor_msgs::JointState
	get_gripper_state();
	
	Eigen::Isometry3d
	get_tf(std::string from, const std::string& to);
	
	geometry_msgs::Pose
	get_pose(const std::string& link, const std::string& ref = "base");
	
	geometry_msgs::Pose
	get_pose_given(const std::string& link, const geometry_msgs::Pose& pose, const std::string& ref = "base");
	
	geometry_msgs::Pose // todo
	get_ee_given_pos(const geometry_msgs::Pose& pose_obj, const Eigen::Isometry3d& offset = Eigen::Translation3d(0.0, 0.0, 0.0) * Eigen::Isometry3d::Identity());
	
	geometry_msgs::Pose // todo
	get_tcp_given_pos(const geometry_msgs::Pose& pose_obj, const Eigen::Isometry3d& offset = Eigen::Translation3d(0.0, 0.0, 0.0) * Eigen::Isometry3d::Identity());
	
	// -- transformations -----------------------------------------------------------
	
	Eigen::Isometry3d // remove?
	w_T_b();

}