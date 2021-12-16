#include <ur5_controllers/joint_position_controller.h>

#include <pluginlib/class_list_macros.hpp>
#include <ur5_dynamics/ur5_dynamics.h>
#include <ur5_description/ur5_description.h>

// export controller
PLUGINLIB_EXPORT_CLASS(ur5_controllers::JointPositionController, controller_interface::ControllerBase)

namespace ur5_controllers
{

bool
JointPositionController::init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle& nh)
{
	// get the joint names defined in config file
	if (not nh.getParam("joint_names", vec_joint_names))
		return false;

	// ensure 6 joint names are loaded
	if (num_joints = vec_joint_names.size(); num_joints != ur5::NUM_JOINTS)
		return false;

	// get the joint_handles
	for (const auto& joint_name : vec_joint_names)
	{
		try
		{
			vec_joints.push_back(hw->getHandle(joint_name));
		}
		catch (const hardware_interface::HardwareInterfaceException& e)
		{
			ROS_ERROR_STREAM("Error getting joint handles: " << e.what());
			return false;
		}
	}

	// read gains and home state from from config
	std::vector<double> vec_kp, vec_kd, vec_home;

	// kp = stiffness
	if (nh.getParam("kp", vec_kp); vec_kp.size() != ur5::NUM_JOINTS)
		return false;
	else
		kp = Eigen::Vector6d(vec_kp.data()).asDiagonal();

	// kd = dampening
	if (nh.getParam("kd", vec_kd); vec_kd.size() != ur5::NUM_JOINTS)
		return false;
	else
		kd = Eigen::Vector6d(vec_kd.data()).asDiagonal();

	// home = initial q
	if (nh.getParam("home", vec_home); vec_home.size() != ur5::NUM_JOINTS)
		return false;
	// set the robot state (qd_d = q_dot_desired)
	else
	{
		q_d   = Eigen::Vector6d(vec_home.data());
		qd_d  = Eigen::Vector6d::Zero();
		qdd_d = Eigen::Vector6d::Zero();
	}

	// subscribe to command
	sub_command = nh.subscribe<ur5_msgs::RobotState>(
		"command",
		1,
		&JointPositionController::callback_command, this,
		ros::TransportHints().tcpNoDelay()
	);

	// publish robot state
	pub_state = nh.advertise<ur5_msgs::RobotState>("robot_state", 1);

	// print gains
	ROS_INFO_STREAM("Initialized UR5 JointPositionController with: kp: " << kp.diagonal().transpose() << ", kd: " << kd.diagonal().transpose());

	return true;
}

void
JointPositionController::starting(const ros::Time& time)
{
	// initialize command buffer
	static ur5_msgs::RobotState state;

	for (auto i = 0; i < ur5::NUM_JOINTS; i++)
		state.q[i] = q_d[i];

	command_buffer.writeFromNonRT(state);
}

void
JointPositionController::update(const ros::Time& /*time*/, const ros::Duration& period)
{
	// elapsed time
	static ros::Duration elapsed_time = ros::Duration(0.);
	elapsed_time += period;

	// get desired joint efforts
	const auto& command = *command_buffer.readFromRT();

	// read the commanded velocity in joint space
	q_d   = Eigen::Vector6d(command.q.data());
	qd_d  = Eigen::Vector6d(command.qd.data());
	qdd_d = Eigen::Vector6d(command.qdd.data());

	// read joint states
	const auto q  = get_position();
	const auto qd = get_velocity();

	// compute dynamics (via kdl)
	const auto g = ur5_dynamics::gravity(q);
	const auto m = ur5_dynamics::mass(q);
	const auto c = ur5_dynamics::coriolis(q, qd);

	// compute desired torque
	const auto y = m * (kp * (q_d - q) + kd * (qd_d - qd) + qdd_d);
	const auto u = y + g + c;

	// set desired command on joint handles
	for (auto i = 0; i < ur5::NUM_JOINTS; ++i)
		vec_joints[i].setCommand(u[i]);

	// publish robot state
	publish_state(q, qd, u);
}

Eigen::Vector6d
JointPositionController::get_position()
{
	static Eigen::Vector6d q;

	for (auto i = 0; i < vec_joints.size(); ++i)
		q[i] = vec_joints[i].getPosition();

	return q;
}

Eigen::Vector6d
JointPositionController::get_velocity()
{
	static Eigen::Vector6d qd;

	for (auto i = 0; i < vec_joints.size(); ++i)
		qd[i] = vec_joints[i].getVelocity();

	return qd;
}

void
JointPositionController::publish_state(const Eigen::Vector6d& q, const Eigen::Vector6d& qd, const Eigen::Vector6d& tau_d)
{
	// set the robot state
	static ur5_msgs::RobotState state;
	for (auto i = 0; i < ur5::NUM_JOINTS; i++)
	{
		state.q[i] = q(i);
		state.qd[i] = qd(i);
		state.tau[i] = tau_d(i);
	}

	// compute pose
	state.pose = ur5_dynamics::fwd_kin<geometry_msgs::Pose>(q);

	pub_state.publish(state);
}

void
JointPositionController::callback_command(const ur5_msgs::RobotStateConstPtr& msg)
{
	command_buffer.writeFromNonRT(*msg);
}

}