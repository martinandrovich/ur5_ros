#include <ur5_gazebo/ur5_gazebo.h>

#include <unordered_map>
#include <ros_utils/gazebo.h>

// Eigen::Isometry3d
// rovi_gazebo::w_T_b()
// {
// 	Eigen::Isometry3d w_T_b;
// 	tf::poseMsgToEigen(rovi_gazebo::get_current_base_pose(), w_T_b);

// 	return w_T_b;
// }

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

geometry_msgs::Pose
ur5_gazebo::get_base_pose()
{
	return gazebo::get_model_state(ur5::ROBOT_NAME);
}

geometry_msgs::Pose
get_pose(const std::string& link, const std::string& reference_frame)
{
	// decode reference frame
	static std::unordered_map<std::string, std::string> map_ref_frames = 
	{
		{"world", "world"},
		// {"base",  ur5::ROBOT_NAME + ur5::LINKS[0]},
	};
	
	// b, base, 0, -> "ur5::base"
	// l1, link1, 1 -> "ur5:link_1"
	
	static std::unordered_map<std::string, int> map_links = 
	{
		{"l6",  1},
		{"ee",  1},
		{"tcp", 1},
	};
	
	if (not map_links.count(link))
		;
		// pose = gazebo::get_link_state("")
	
	
	
	// link = "ur5::" + ur5::LINKS("link");
	
	// if lookuo doesn't exist, return what Gazebo gives
	;
	
	// lookup in lambda map
	;
}