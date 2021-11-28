cmake_minimum_required(VERSION 3.4)
project(ur5_controllers)

# C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# find system libraries
find_package(Eigen3 REQUIRED)

# find catkin and any catkin packages
find_package(
	catkin REQUIRED
	COMPONENTS
		roscpp
		roslib
		ros_utils
		std_msgs
		xacro
		gazebo_ros
		controller_manager
		controller_interface
		effort_controllers
		joint_state_controller
		joint_state_publisher
		robot_state_publisher
		pluginlib
		ur5_msgs
		ur5_description
		ur5_dynamics
		ur5_gazebo
)

# declare a catkin package
catkin_package(
	INCLUDE_DIRS
		include
		${EIGEN3_INCLUDE_DIR}
	LIBRARIES
		ur5_controllers
	CATKIN_DEPENDS
		roscpp
		roslib
		ros_utils
		std_msgs
		xacro
		gazebo_ros
		controller_manager
		controller_interface
		effort_controllers
		joint_state_controller
		joint_state_publisher
		robot_state_publisher
		pluginlib
		ur5_msgs
		ur5_description
		ur5_dynamics
		ur5_gazebo
	DEPENDS
		EIGEN3
)

# include directories
include_directories(
	include
	${catkin_INCLUDE_DIRS}
	${EIGEN3_INCLUDE_DIRS}
)

################################################################################################

# libraries
# REMEMBER TO ADD TO INSTALL TARGETS (bottom of file) !!!

## ur5_controller
add_library(ur5_controllers
		src/lib_ur5_controllers.cpp # interface
		src/joint_position_pd_gravity_controller.cpp
		src/joint_position_controller.cpp
)
target_link_libraries(ur5_controllers ${catkin_LIBRARIES})

################################################################################################

# nodes

add_executable(demo_ur5_controllers examples/demo_ur5_controllers.cpp)
target_link_libraries(demo_ur5_controllers ${catkin_LIBRARIES} ur5_controllers)

################################################################################################

# install targets

## executables and/or libraries for installation
install(
	TARGETS
		ur5_controllers
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

## install header files
install(
	DIRECTORY
		include/ur5_controllers/
	DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

## install ros_control plugin
install(FILES ur5_controllers_plugin.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})