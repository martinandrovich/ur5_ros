#pragma once

#include <string>
#include <Eigen/Eigen>

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Pose.h>
#include <ur5_description/ur5_description.h>

namespace ur5
{
	
	// -- state and pose ------------------------------------------------------------
	
	sensor_msgs::JointState
	get_state();
	
	Eigen::Vector6d
	q();

	sensor_msgs::JointState
	get_ee_state();
	
	Eigen::Isometry3d
	get_tf(std::string from, const std::string& to);
	
	geometry_msgs::Pose
	get_pose(const std::string& link, const std::string& ref = "base");
	
	geometry_msgs::Pose
	get_pose_given(const std::string& link, const geometry_msgs::Pose& pose, const std::string& ref = "base");
	
	geometry_msgs::Pose
	get_ee_given_obj_pose(const geometry_msgs::Pose& pose_obj, const Eigen::Isometry3d& offset = Eigen::Isometry3d::Identity());
	
	geometry_msgs::Pose // todo
	get_tcp_given_pose(const geometry_msgs::Pose& pose_obj, const Eigen::Isometry3d& offset = Eigen::Isometry3d::Identity());
	
	// -- transformations -----------------------------------------------------------
	
	Eigen::Isometry3d
	w_T_b();

}