#include <ur5_gazebo/ur5_gazebo.h>

#include <unordered_map>
#include <list>
#include <functional>

#include <eigen_conversions/eigen_msg.h>
#include <ur5_description/ur5.h>
#include <ros_utils/gazebo.h>
#include <ros_utils/std.h>

sensor_msgs::JointState
ur5_gazebo::get_robot_state()
{
	auto joint_states = *(ros::topic::waitForMessage<sensor_msgs::JointState>(ur5::JOINT_STATE_TOPIC, ros::Duration(1.0)));

	// filter out joint values (only keep first six joints)
	joint_states.name.resize(ur5::NUM_JOINTS);
	joint_states.position.resize(ur5::NUM_JOINTS);
	joint_states.velocity.resize(ur5::NUM_JOINTS);
	joint_states.effort.resize(ur5::NUM_JOINTS);

	return joint_states;
}

sensor_msgs::JointState
ur5_gazebo::get_gripper_state()
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
ur5_gazebo::get_tf(std::string from, const std::string& to)
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
		return gazebo::get_tf(from, ur5::LINKS["l6"]) * ur5::l6_T_ee * ur5::ee_T_tcp;

	else
		return gazebo::get_tf(from, ur5::LINKS[to]);
}

geometry_msgs::Pose
ur5_gazebo::get_pose(const std::string& link, const std::string& ref)
{
	auto tf = ur5_gazebo::get_tf(ref, link); 
	auto pose = geometry_msgs::Pose();
	tf::poseEigenToMsg(tf, pose);
	
	return pose;
}

Eigen::Isometry3d
ur5_gazebo::w_T_b()
{
	return ur5_gazebo::get_tf("w", "b");
}