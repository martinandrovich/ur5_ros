#include <ur5_gazebo/ur5_gazebo.h>

#include <unordered_map>
#include <list>
#include <functional>

#include <eigen_conversions/eigen_msg.h>
#include <ur5_description/ur5_description.h>
#include <ros_utils/gazebo.h>
#include <ros_utils/std.h>
#include <ros_utils/eigen.h>
#include <ros_utils/geometry_msgs.h>

// -- state and pose ------------------------------------------------------------



sensor_msgs::JointState
ur5::get_state()
{
	auto joint_states = *(ros::topic::waitForMessage<sensor_msgs::JointState>(ur5::JOINT_STATE_TOPIC, ros::Duration(1.0)));

	// filter out joint values (only keep first six joints)
	joint_states.name.resize(ur5::NUM_JOINTS);
	joint_states.position.resize(ur5::NUM_JOINTS);
	joint_states.velocity.resize(ur5::NUM_JOINTS);
	joint_states.effort.resize(ur5::NUM_JOINTS);

	return joint_states;
}

Eigen::Vector6d
ur5::q()
{
	static Eigen::Vector6d q;
	const auto joint_state = ur5::get_state();
	
	for (auto i = 0; i < ur5::NUM_JOINTS; i++)
		q[i] = joint_state.position[i];
	
	return q;
}

sensor_msgs::JointState
ur5::get_ee_state()
{
	auto joint_states = *(ros::topic::waitForMessage<sensor_msgs::JointState>(ur5::JOINT_STATE_TOPIC, ros::Duration(1.0)));

	// filter out joint values (remove first six joints)
	joint_states.name.erase(joint_states.name.begin(), joint_states.name.begin() + ur5::NUM_JOINTS);
	joint_states.position.erase(joint_states.position.begin(), joint_states.position.begin() + ur5::NUM_JOINTS);
	joint_states.velocity.erase(joint_states.velocity.begin(), joint_states.velocity.begin() + ur5::NUM_JOINTS);
	joint_states.effort.erase(joint_states.effort.begin(), joint_states.effort.begin() + ur5::NUM_JOINTS);

	return joint_states;
}

Eigen::Isometry3d
ur5::get_tf(std::string from, const std::string& to)
{
	// frames: w, b, l1, l2, l3, l4, l5, l6, ee*, tcp*
	// usage: auto w_T_l6 = get_tf("w", "l6");
	
	// decode reference frame
	if (is_in(from, { "world", "w", "" }))
		from = "world";
	else
		from = ur5::LINKS[from];

	// return appropritate transformation
	if (to == "ee")
		return gazebo::get_tf(from, ur5::LINKS["l6"]) * ur5::l6_T_ee;
		
	else
	if (to == "tcp")
		return gazebo::get_tf(from, ur5::LINKS["l6"]) * ur5::l6_T_ee * ur5::ee_T_tcp();

	else
		return gazebo::get_tf(from, ur5::LINKS[to]);
}

geometry_msgs::Pose
ur5::get_pose(const std::string& link, const std::string& ref)
{
	auto tf = ur5::get_tf(ref, link); 
	auto pose = geometry_msgs::Pose();
	tf::poseEigenToMsg(tf, pose);
	
	return pose;
}

geometry_msgs::Pose
ur5::get_ee_given_pose(const geometry_msgs::Pose& pose, const Eigen::Isometry3d& offset)
{
	// given pose in world frame (w_T_obj), compute the desired EE pose in robot base frame (b_T_ee)
	// offset = obj_T_offset is an offset to the specified pose
	
	Eigen::Isometry3d b_T_ee, w_T_obj;
	w_T_obj = Eigen::make_tf(pose) * offset;
	
	// w_T_ee = w_T_obj
	// w_T_ee = w_T_b * b_T_ee -> b_T_ee = inv(w_T_b) * w_T_ee -> b_T_ee = inv(w_T_b) * w_T_obj
	
	b_T_ee = w_T_b().inverse() * w_T_obj;

	return geometry_msgs::make_pose(b_T_ee);
}

geometry_msgs::Pose
ur5::get_ee_given_pose_at_tcp(const geometry_msgs::Pose& pose, const Eigen::Isometry3d& offset)
{
	// compute the desired EE pose such that TCP is at the given pose in world frame (w_T_obj) 
	// offset = obj_T_offset is an offset to the specified pose
	
	return ur5::get_ee_given_pose(pose, offset * ur5::ee_T_tcp().inverse());
}

// -- transformations -----------------------------------------------------------

Eigen::Isometry3d
ur5::w_T_b()
{
	return ur5::get_tf("w", "b");
}