#include "moveit_msgs/PlanningScene.h"
#include "ur5_planner/moveit.h"

#include <string>
#include <string_view>
#include <vector>
#include <atomic>
#include <mutex>
#include <thread>
#include <memory>

#include <ros/ros.h>
#include <ros_utils/moveit.h>
#include <ros_utils/std.h>

#include <ur5_description/ur5_description.h>
#include <ur5_gazebo/ur5_gazebo.h>

#include <pluginlib/class_loader.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>

namespace ur5::moveit
{
	// init bool
	std::atomic<bool> is_init = false;

	// planning plugin
	static inline robot_model::RobotModelPtr robot_model = nullptr;
	static inline robot_model_loader::RobotModelLoaderPtr robot_model_loader = nullptr;
	static inline boost::shared_ptr<pluginlib::ClassLoader<planning_interface::PlannerManager>> planner_plugin_loader = nullptr;
	static inline planning_interface::PlannerManagerPtr planner_instance = nullptr;

	// planning scene and collision objects
	static inline std::recursive_mutex mtx_planning_scene;
	static inline planning_scene::PlanningScenePtr planning_scene;
	static inline std::vector<moveit_msgs::CollisionObject> vec_cobjs;
	static inline std::thread* thread_planning_scene_pub = nullptr;

	// publisher(s)
	static inline ros::Publisher pub_planning_scene;

	void
	check_init();
}

inline void
ur5::moveit::check_init()
{
	if (not is_init)
		throw std::runtime_error("ur5::moveit has not been initialized before use.");
};

bool
ur5::moveit::init(ros::NodeHandle& nh)
{
	if (is_init)
		return true;

	// setup async spinners for moveit
	ros::AsyncSpinner spin(0);
	spin.start();

	// create the model of the robot
	robot_model_loader = std::make_shared<robot_model_loader::RobotModelLoader>(ur5::ROBOT_DESCRIPTION);
	robot_model = robot_model_loader->getModel();

	// create planning scene
	planning_scene = std::make_shared<planning_scene::PlanningScene>(robot_model);

	// create planner objects
	planner_plugin_loader.reset(new pluginlib::ClassLoader<planning_interface::PlannerManager>("moveit_core", "planning_interface::PlannerManager"));
	planner_instance.reset(planner_plugin_loader->createUnmanagedInstance(PLANNER_PLUGIN));

	// try to initialize
	if (not planner_instance->initialize(robot_model, ros::this_node::getNamespace()))
		throw std::runtime_error("Could not initialize ur5::moveit planner instance...");

	is_init = true;
	ROS_INFO_STREAM("Using planning interface '" << planner_instance->getDescription() << "'...");

	// create planning scene publisher
	pub_planning_scene = nh.advertise<moveit_msgs::PlanningScene>(PLANNING_SCENE_TOPIC, 1);

	return is_init;
}

void
ur5::moveit::terminate()
{
	check_init();

	ROS_INFO("Terminating ur5::moveit...");

	// bye bye
	is_init = false;

	// scene publisher + thread (if started)
	if (thread_planning_scene_pub)
		thread_planning_scene_pub->join();

	pub_planning_scene.shutdown();
	delete thread_planning_scene_pub;

	// planner plugin
	planner_instance->terminate();
	planner_instance.reset();
	planner_plugin_loader.reset();

	// planning scene
	planning_scene.reset();

	// robot model + loader
	robot_model.reset();
	robot_model_loader.reset();

	ROS_INFO("ur5::moveit finished cleanly!");
}

void
ur5::moveit::update_planning_scene_from_gazebo(bool remove_attached_cobjs)
{
	check_init();

	// lock planning scene recursive mutex for this scope
	std::lock_guard lock(mtx_planning_scene);

	// get robot state to modify
	auto& robot_state = planning_scene->getCurrentStateNonConst();

	// update arm group
	const auto q_ur5 = ur5::get_state().position;
	robot_state.setJointGroupPositions(ARM_GROUP, q_ur5);

	// set robot base pose to the current one from Gazebo using the virtual floating joint of RobotState
	const auto pose_base = ur5::get_pose("b", "w");
	ur5::moveit::move_base(pose_base);

	// add the collisions to the scene; remove any attached objects if specified
	vec_cobjs = ::moveit::get_gazebo_cobjs(planning_scene->getPlanningFrame(), gazebo_cobj_exclude);
	::moveit::add_cobjs(planning_scene, vec_cobjs, remove_attached_cobjs);

	// update ee group
	if (ur5::has_ee())
	{
		const auto q_ee = ur5::get_ee_state().position;
		robot_state.setJointGroupPositions(EE_GROUP, q_ee);
	}
}

planning_scene::PlanningScenePtr
ur5::moveit::get_mutexed_planning_scene()
{
	check_init();

	// return a mutexed shared pointer to the planning scene
	// when the shared pointer goes out of scope, the mutex lock is released
	// https://stackoverflow.com/a/50165107/1658105

	auto counter = std::make_shared<std::lock_guard<std::recursive_mutex>>(mtx_planning_scene);
	std::shared_ptr<planning_scene::PlanningScene> ptr{counter, &*planning_scene};

	return ptr;
}

void
ur5::moveit::publish_planning_scene()
{
	check_init();

	std::lock_guard lock(mtx_planning_scene);

	static moveit_msgs::PlanningScene planning_scene_msg;
	planning_scene->getPlanningSceneMsg(planning_scene_msg);
	pub_planning_scene.publish(planning_scene_msg);
}

void
ur5::moveit::print_planning_scene()
{
	check_init();

	// lock planning scene recursive mutex for this scope
	std::lock_guard lock(mtx_planning_scene);

	// get planning scene msg
	static moveit_msgs::PlanningScene planning_scene_msg;
	planning_scene->getPlanningSceneMsg(planning_scene_msg);

	std::cout << "\nrobot state:\n";
	std::cout << planning_scene_msg.robot_state.joint_state.position << std::endl;

	std::cout << "\ncollision objects:\n";
	for (const auto& cobj : planning_scene_msg.world.collision_objects)
		std::cout << "- " << cobj.id << "\n";

	std::cout << "\nattached collison objects:\n";
	for (const auto& acobj : planning_scene_msg.robot_state.attached_collision_objects)
		std::cout << "- " << acobj.object.id << "\n";
}

void
ur5::moveit::start_scene_publisher(double freq)
{
	check_init();

	if (thread_planning_scene_pub != nullptr)
		return;

	ROS_INFO_STREAM("Starting MoveIt planning scene publisher on topic: '" << PLANNING_SCENE_TOPIC << "' at " << freq << " Hz.");

	thread_planning_scene_pub = new std::thread([&]()
	{
		ros::Rate lp(freq); // Hz
		while(ros::ok() and is_init)
		{
			// works with recursive mutex since a different thread is invoking the method
			publish_planning_scene();
			lp.sleep();
		}

		ROS_INFO("Exiting scene publisher thread...");
	}); // .join() in ur5::moveit::terminate();
};

void
ur5::moveit::attach_object_to_ee(const std::string& name)
{
	check_init();

	// lock planning scene recursive mutex for this scope
	std::lock_guard lock(mtx_planning_scene);

	// find collision object
	const auto cobj = std::find_if(vec_cobjs.begin(), vec_cobjs.end(), [&](auto& obj) {
		return obj.id == name;
	});

	if (cobj == vec_cobjs.end())
		throw std::runtime_error("The collision object " + name + " does not exist in the current scene.");

	// get current planning scence (msg)
	static moveit_msgs::PlanningScene planning_scene_msg;
	planning_scene->getPlanningSceneMsg(planning_scene_msg);

	// attach the object to the end-effector
	moveit_msgs::AttachedCollisionObject acobj;
	acobj.link_name = ur5::LINKS.URDF("end-effector");
	acobj.object = moveit_msgs::CollisionObject(*cobj);
	acobj.object.mesh_poses[0].position.z += 0.005; // offset to avoid collisions when planning
	acobj.object.operation = moveit_msgs::CollisionObject::ADD;

	planning_scene_msg.robot_state.attached_collision_objects = { acobj };

	// remove object from scene
	cobj->operation = moveit_msgs::CollisionObject::REMOVE;
	planning_scene_msg.world.collision_objects = vec_cobjs;

	// update robot state
	// auto& robot_state = planning_scene->getCurrentStateNonConst();
	// const auto q_ur5 = rovi_gazebo::get_current_robot_state().position;
	// robot_state.setJointGroupPositions(ARM_GROUP, q_ur5);

	// update planning scene
	planning_scene->setPlanningSceneMsg(planning_scene_msg);
}

void
ur5::moveit::move_base(const geometry_msgs::Pose& pose)
{
	check_init();

	// lock planning scene recursive mutex for this scope
	std::lock_guard lock(mtx_planning_scene);

	// get robot state to modify
	auto& robot_state = planning_scene->getCurrentStateNonConst();

	// set robot bases pose using the virtual floating joint of RobotState
	::moveit::set_floating_jnt_pose(robot_state, "world_offset", pose);
}

planning_interface::MotionPlanResponse
ur5::moveit::plan(const geometry_msgs::Pose& pose_des, const Planner& planner, double max_planning_time, size_t max_planning_attempts)
{
	check_init();

	// plan for a desired end-effector (flange) pose (b_T_ee)
	// https://ros-planning.github.io/moveit_tutorials/doc/motion_planning_api/motion_planning_api_tutorial.html

	// lock planning scene recursive mutex for this scope
	std::lock_guard lock(mtx_planning_scene);

	// resolve and set planner
	ros::param::set(ARM_GROUP + "/default_planner_config", PLANNERS[planner]);

	// desired pose with timestamp
	geometry_msgs::PoseStamped pose;
	pose.header.frame_id = ur5::LINKS.URDF("base");
	pose.pose = pose_des;

	// specify constraints for desired link, pose and tolerances
	std::vector<double> tol_pos(3, 0.001); // [m]
	std::vector<double> tol_ori(3, 0.001); // [rad]
	const auto pose_constraints = kinematic_constraints::constructGoalConstraints(ur5::LINKS.URDF("end-effector"), pose, tol_pos, tol_ori);

	// specify planning request (plan group etc.)
	// ARM_GROUP is used; if EE is to be included in collision checks, the EE group must be defined
	// as an end-effector with ARM_GROUP as its parent

	planning_interface::MotionPlanRequest req;
	req.group_name = ARM_GROUP;
	req.allowed_planning_time = max_planning_time; // default: 1.0
	req.num_planning_attempts = max_planning_attempts; // default: 10
	req.max_acceleration_scaling_factor = 1.0;
	req.max_velocity_scaling_factor = 1.0;
	req.goal_constraints.push_back(pose_constraints);

	// get planning context and plan
	planning_interface::MotionPlanResponse res;
	auto context = planner_instance->getPlanningContext(planning_scene, req, res.error_code_);
	context->solve(res);

	if (res.error_code_.val != res.error_code_.SUCCESS)
		throw std::runtime_error("Planner failed in ur5::moveit::plan() [" + std::to_string(res.error_code_.val) + "]");

	// clean up
	context->terminate();
	context.reset(); // delete ptr

	// return plan (MotionPlanResponse)
	return res;
}

trajectory_msgs::JointTrajectory
ur5::moveit::plan_to_jnt_traj(planning_interface::MotionPlanResponse& plan, double dt, double tol, double max_vel_scale, double max_acc_scale)
{
	check_init();

	// parameterize kinematic trajectories for velocity and acceleration
	// https://ros-planning.github.io/moveit_tutorials/doc/time_parameterization/time_parameterization_tutorial.html

	trajectory_processing::TimeOptimalTrajectoryGeneration totg(tol, dt);
	totg.computeTimeStamps(*plan.trajectory_, max_vel_scale, max_acc_scale);
	const robot_trajectory::RobotTrajectory& traj = *plan.trajectory_;

	// define joint trajectory
	trajectory_msgs::JointTrajectory jnt_traj;
	jnt_traj.joint_names = ur5::JNT_NAMES;

	// iterate waypoints and create JointTrajectoryPoint
	for (auto [i, t] = std::make_tuple(0, 0.0); i < traj.getWayPointCount(); i++, t += dt)
	{
		// get robot state of current waypoint
		const robot_state::RobotState& state = traj.getWayPoint(i);

		// get joint values for arm group and copy into JointTrajectoryPoint
		static trajectory_msgs::JointTrajectoryPoint pt;

		pt.time_from_start = ros::Duration(t);
		state.copyJointGroupPositions(ARM_GROUP, pt.positions);         // q
		state.copyJointGroupVelocities(ARM_GROUP, pt.velocities);       // qd
		state.copyJointGroupAccelerations(ARM_GROUP, pt.accelerations); // qdd

		jnt_traj.points.push_back(pt);
	}

	return jnt_traj;
}

void
ur5::moveit::test_get_mutexed_planning_scene()
{
	using namespace std::chrono_literals;
	
	constexpr auto NUM_ATTEMPTS = 20;
	std::atomic<int> counter = 0;
	
	// one thread to test mutex
	auto t1 = std::thread([&]
	{
		while (++counter < NUM_ATTEMPTS)
		{
			std::this_thread::sleep_for(0.05s);
			auto success = mtx_planning_scene.try_lock();
			ROS_INFO_STREAM("t1: Trying to lock: " << std::boolalpha << success);
			std::this_thread::sleep_for(0.1s);
			if (not success) continue;
			
			ROS_INFO_STREAM("t1: Got lock!");
			std::this_thread::sleep_for(0.5s);
			mtx_planning_scene.unlock();
			ROS_INFO_STREAM("t1: Releasing lock...");
		};
	});
	
	// one thread to use mutexed planning scene
	auto t2 = std::thread([&]
	{
		while (++counter < NUM_ATTEMPTS)
		{
			std::this_thread::sleep_for(0.05s);
			ROS_INFO_STREAM("t2: Getting scene...");
			auto ps = ur5::moveit::get_mutexed_planning_scene();
			ROS_INFO_STREAM("t2: Got scene! Using it: " << ps->getRobotModel()->getName());
			std::this_thread::sleep_for(0.3s);
			ROS_INFO_STREAM("t2: Releasing scene...");
		}; // lock released here
	});
	
	t1.join();
	t2.join();
}