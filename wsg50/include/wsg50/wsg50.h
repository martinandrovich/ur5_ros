#pragma once

#include <string>
#include <std_msgs/Float64.h>

namespace wsg50
{

	void
	grasp(bool block = false);

	void
	release(bool block = false);
	
	void // todo
	get_state();

	inline const std::string COMMAND_TOPIC  = "/wsg50_joint_effort_controller/command";
	inline const double      EFFORT_GRASP   = -10.0; // [Nm]
	inline const double      EFFORT_RELEASE =  10.0; // [Nm]
	inline const auto        NUM_JOINTS     = 2;

}
