#pragma once

#include <vector>
#include <Eigen/Eigen>

#include <ros/ros.h>
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <realtime_tools/realtime_buffer.h>

#include <ur5_description/ur5.h>
#include <ur5_msgs/RobotState.h>

namespace ur5_controllers
{

class JointPositionController final: public controller_interface::Controller<hardware_interface::EffortJointInterface>
{

public:

	JointPositionController() = default;
	~JointPositionController() { sub_command.shutdown(); }

	bool
	init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle& nh) override;

	void
	starting(const ros::Time& time) override;

	void
	update(const ros::Time& /*time*/, const ros::Duration& /*period*/) override;

private:

	ros::Subscriber sub_command;
	ros::Publisher pub_state;
	
	size_t num_joints;
	std::vector<std::string> vec_joint_names;
	std::vector<hardware_interface::JointHandle> vec_joints;
	realtime_tools::RealtimeBuffer<ur5_msgs::RobotState> command_buffer;

	Eigen::Matrix6d kd;
	Eigen::Matrix6d kp;

	Eigen::Vector6d q_d;
	Eigen::Vector6d qd_d;
	Eigen::Vector6d qdd_d;

	Eigen::Vector6d
	get_position();

	Eigen::Vector6d
	get_velocity();
	
	void
	callback_command(const ur5_msgs::RobotStateConstPtr& msg);

	void
	publish_state(const Eigen::Vector6d& q, const Eigen::Vector6d& qd, const Eigen::Vector6d& tau_d);

};

}