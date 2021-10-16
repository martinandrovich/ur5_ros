#include <wsg50/joint_effort_controller.h>
#include <ros/transport_hints.h>

// export controller
PLUGINLIB_EXPORT_CLASS(wsg50::JointEffortController, controller_interface::ControllerBase)

namespace wsg50
{
	bool
	JointEffortController::init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle& nh)
	{
		// get the joint names defined in config file
		if (not nh.getParam("joint_names", vec_joint_names))
			return false;

		// ensure 2 joint names are loaded
		if (num_joints = vec_joint_names.size(); num_joints != 2)
			return false;

		// get joint handles
		for (const auto& joint_name : vec_joint_names)
		{
			try
			{
				vec_joints.push_back(hw->getHandle(joint_name));
			}
			catch (const hardware_interface::HardwareInterfaceException& e)
			{
				ROS_ERROR_NAMED(CONTROLLER_NAME, "Error getting joint handles: %s", e.what());
				return false;
			}
		}

		// initialize command buffer
		std_msgs::Float64 msg; msg.data = 0;
		commands_buffer.writeFromNonRT(msg);

		// subscribe to command
		sub_command = nh.subscribe<std_msgs::Float64>(
			"command",
			1,
			&JointEffortController::callback_command, this,
			ros::TransportHints().udp().tcpNoDelay()
		);

		// init complete
		ROS_INFO_STREAM_NAMED(CONTROLLER_NAME, "Loaded " << CONTROLLER_NAME);

		return true;
	}

	void
	JointEffortController::starting(const ros::Time& time)
	{
		// command initial joint values
		for (auto i = 0; i < 2; i++)
			vec_joints[i].setCommand(TAU_INIT[i]);
	}

	void
	JointEffortController::update(const ros::Time& time, const ros::Duration& dur)
	{
		// get commanded joint efforts
		const auto& command = *commands_buffer.readFromRT();
		
		// compute desired torque
		Eigen::Vector2d tau_des(command.data, command.data);

		// saturate rotatum
		if (SATURATE_ROTATUM)
			tau_des = saturate_rotatum(tau_des);

		for (auto i = 0; i < 2; i++)
			vec_joints[i].setCommand(tau_des(i));
	}

	void
	JointEffortController::callback_command(const std_msgs::Float64ConstPtr& msg)
	{
		// set commanded torque
		// ROS_WARN_STREAM(msg->data);
		commands_buffer.writeFromNonRT(*msg);
	}

	Eigen::Vector2d
	JointEffortController::get_position()
	{
		static Eigen::Vector2d q;

		for (auto i = 0; i < vec_joints.size(); ++i)
			q[i] = vec_joints[i].getPosition();

		return q;
	}
	
	
	Eigen::Vector2d
	JointEffortController::saturate_rotatum(const Eigen::Vector2d& tau_des, const double period)
	{
		// previous desired torque and saturated torque
		static Eigen::Vector2d tau_des_prev = Eigen::Vector2d::Zero();
		static Eigen::Vector2d tau_des_sat = Eigen::Vector2d::Zero();

		// compute saturated torque
		for (auto i = 0; i < tau_des_sat.size(); ++i)
		{
			const double tau_dot = (tau_des[i] - tau_des_prev[i]) / period;
			tau_des_sat[i] = tau_des_prev[i] + std::max(std::min(tau_dot, TAU_DOT_MAX * period), -(TAU_DOT_MAX * period));
		}

		// save for next iteration and return
		tau_des_prev = tau_des_sat;

		return tau_des_sat;
	}

}