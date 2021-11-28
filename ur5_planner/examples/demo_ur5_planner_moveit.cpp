#include <iostream>

#include <ros/ros.h>
#include <ros_utils/ros_utils.h>

#include <wsg50/wsg50.h>
#include <ur5_planner/moveit.h>
#include <ur5_gazebo/ur5_gazebo.h>
#include <ur5_dynamics/ur5_dynamics.h>
#include <ur5_controllers/ur5_controllers.h>

#include <trajectory_msgs/JointTrajectoryPoint.h>

int main(int argc, char** argv)
{
	ros::init(argc, argv, "demo_ur5_planner_moveit");
	ros::NodeHandle nh;

	// first run:
	// roslaunch ur5_gazebo ur5.launch ee:=wsg50 moveit:=true rviz:=true pos:="-z 0.7"

	// WARNING: can go wrong if planner finds a path that collides with ground

	// initialize moveit planner
	ur5::moveit::init(nh);
	ur5::moveit::start_scene_publisher(100); // Hz

	// ------------------------------------------------------------------------------

	ENTER_TO_CONTINUE("load scene from Gazebo");

	// add some models to Gazebo
	auto pose_bottle = geometry_msgs::make_pose({0.5, 0.5, 0});
	gazebo::spawn_model("bottle", "bottle1", pose_bottle);

	// load collision objects from Gazebo
	ur5::moveit::gazebo_cobj_exclude = { "ur5", "ground_plane", "camera_stereo", "openni_kinect", "projector" };
	ur5::moveit::update_planning_scene_from_gazebo();
	ur5::moveit::publish_planning_scene();

	// ------------------------------------------------------------------------------

	ENTER_TO_CONTINUE("demonstrate EE");

	wsg50::grasp(true); // block while commanding
	ur5::moveit::update_planning_scene_from_gazebo();

	wsg50::release(true); // block while commanding
	ur5::moveit::update_planning_scene_from_gazebo();

	// ------------------------------------------------------------------------------

	ENTER_TO_CONTINUE("plan");

	// define pose of bottle in robot base frame
	auto offset = Eigen::make_tf({0, 0, 0.1}) * ur5::ee_T_tcp.inverse();
	auto b_T_ee = ur5::get_ee_given_pose(pose_bottle, offset);

	// set planner property (optional)
	// ur5_ros/ur5_moveit_config/config/ompl_planning.yaml
	ur5::moveit::set_planner_config(Planner::RRT, "goal_bias", 0.01);

	// make plan
	auto plan = ur5::moveit::plan(b_T_ee, Planner::RRT, 1.0);
	auto traj = ur5::moveit::plan_to_jnt_traj(plan, ur5::EXEC_DT);

	// check joint names (optional)
	std::cout << "traj.joint_names: " << traj.joint_names << std::endl;

	// ------------------------------------------------------------------------------

	ENTER_TO_CONTINUE("execute trajectory");

	ur5::command_traj(traj); // block while commanding
	ur5::moveit::update_planning_scene_from_gazebo();
	ur5::moveit::print_planning_scene();

	// ------------------------------------------------------------------------------

	ENTER_TO_CONTINUE("attach object to EE");

	ur5::moveit::attach_object_to_ee("bottle1");
	ur5::moveit::print_planning_scene();

	// ------------------------------------------------------------------------------

	ENTER_TO_CONTINUE("grasp object");

	wsg50::grasp(true); // block while commanding
	ur5::moveit::update_planning_scene_from_gazebo(false); // keep attached objects
	ur5::moveit::print_planning_scene();

	// ------------------------------------------------------------------------------

	ENTER_TO_CONTINUE("move object upwards");

	// plan using IK to move object
	pose_bottle.position.z += 0.5;
	b_T_ee = ur5::get_ee_given_pose(pose_bottle, offset);
	auto q_d = ur5_dynamics::inv_kin(b_T_ee, ur5::q());

	// command robot to setpoint (joint state)
	ur5::command_setpoint(q_d); // block while commanding
	ur5::moveit::update_planning_scene_from_gazebo(false); // keep attached objects

	// ------------------------------------------------------------------------------

	ENTER_TO_CONTINUE("test mutexed planning scene");
	
	// test whether the mutexed planning scene can be safely aqcuired while being published by another thread
	ur5::moveit::test_get_mutexed_planning_scene();

	// ------------------------------------------------------------------------------

	ENTER_TO_CONTINUE("terminate");

	ur5::moveit::terminate();
	return 0;
}
