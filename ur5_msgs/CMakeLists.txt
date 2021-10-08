cmake_minimum_required(VERSION 3.4)
project(ur5_msgs)

# find catkin and any catkin packages
find_package(
	catkin REQUIRED
	COMPONENTS
		message_generation
		std_msgs
		geometry_msgs
		actionlib_msgs
)

add_message_files(
	FILES
		RobotState.msg
		PoseTwist.msg
)

# add_service_files(FILES)
# add_action_files(FILES)

generate_messages(
	DEPENDENCIES
		std_msgs
		geometry_msgs
		actionlib_msgs
)

# declare a catkin package
catkin_package(
	CATKIN_DEPENDS
		message_runtime
		std_msgs
		geometry_msgs
		actionlib_msgs
)
