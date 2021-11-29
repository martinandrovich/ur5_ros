#include <iostream>
#include <vector>

#include <ros/ros.h>
#include <ros_utils/ros_utils.h>
#include <ur5_planner/moveit.h>
#include <ur5_planner/reachability.h>
#include <ur5_description/ur5_description.h>
#include <geometry_msgs/Pose.h>

int main(int argc, char** argv)
{
	ros::init(argc, argv, "demo_ur5_planner_reachability");
	ros::NodeHandle nh;

	// first run:
	// roslaunch ur5_gazebo ur5.launch ee:=wsg50 gazebo:=false moveit:=true rviz:=true

	// this example requires the 'rovi_models' package for collision objects meshes

	// ------------------------------------------------------------------------------

	ENTER_TO_CONTINUE("setup planning scene");

	// init
	ur5::moveit::init(nh);
	ur5::moveit::start_scene_publisher(100);

	// define poses
	auto pose_base   = geometry_msgs::make_pose({ 0.25, 0.25, 0.75 });
	auto pose_table  = geometry_msgs::make_pose({ 0.40, 0.60, 0.64 });
	auto pose_bottle = geometry_msgs::make_pose({ 0.50, 1.00, 0.75 });

	// add collision objects to planning scene
	ur5::moveit::add_cobjs(
	{
		{ "table", pose_table },
		{ "bottle", pose_bottle }
	}, "rovi_models");

	// ------------------------------------------------------------------------------

	ENTER_TO_CONTINUE("do reachability test");

	// define grasping orientation
	auto T_grasp = GRASP_FROM_SIDE_AT_TCP(0.1); // = Eigen::make_tf({ 0, 0, z }) * ur5::ee_T_tcp.inverse()
	// auto T_grasp = GRASP_TOP_AT_TCP(0.2);  // = Eigen::make_tf({ 0, 0, z }, { -M_PI_2, 0, 0 }) * ur5::ee_T_tcp.inverse();

	// perform reachability test (and visualize in RVIZ)
	auto result = ur5::moveit::reachability(pose_base, pose_bottle, T_grasp, 16, VISUALIZE_REACHABILITY);

	// ------------------------------------------------------------------------------

	ENTER_TO_CONTINUE("terminate");
	ur5::moveit::terminate();

	return 0;
}