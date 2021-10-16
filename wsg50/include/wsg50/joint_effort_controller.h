#pragma once

#include <string>
#include <vector>
#include <Eigen/Eigen>

#include <ros/ros.h>
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <realtime_tools/realtime_buffer.h>
#include <std_msgs/Float64.h>

namespace wsg50
{
	class JointEffortController final: public controller_interface::Controller<hardware_interface::EffortJointInterface>
	{
		public:

			static inline constexpr auto CONTROLLER_NAME = "JointEffortController";
			static inline constexpr auto SATURATE_ROTATUM = true;
			static inline constexpr auto TAU_DOT_MAX = 50.0;
			static inline const std::vector<double> TAU_INIT = { 0, 0 };

			JointEffortController() = default;
			~JointEffortController() { sub_command.shutdown(); }

			bool
			init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle& nh) override;

			void
			starting(const ros::Time& time) override;

			void
			update(const ros::Time& /*time*/, const ros::Duration& /*period*/) override;

		private:
			
			ros::Subscriber sub_command;
			size_t num_joints;
			std::vector<std::string> vec_joint_names;
			std::vector<hardware_interface::JointHandle> vec_joints;
			realtime_tools::RealtimeBuffer<std_msgs::Float64> commands_buffer;

			void
			callback_command(const std_msgs::Float64ConstPtr& msg);

			Eigen::Vector2d
			get_position();
			
			Eigen::Vector2d 
			saturate_rotatum(const Eigen::Vector2d& tau_des, const double period = 0.001 /* [s] */);
	};

}
