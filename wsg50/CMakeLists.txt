cmake_minimum_required(VERSION 3.4)
project(wsg50)

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
)

# declare a catkin package
catkin_package(
	INCLUDE_DIRS
		include
		${EIGEN3_INCLUDE_DIR}
	LIBRARIES
		wsg50
		wsg50_controllers
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

## wsg50_controllers
add_library(
	wsg50_controllers
		src/joint_effort_controller.cpp
)
target_link_libraries(wsg50_controllers ${catkin_LIBRARIES})

## wsg50_interface
add_library(wsg50 src/lib_wsg50.cpp)
target_link_libraries(wsg50 ${catkin_LIBRARIES})

################################################################################################

# nodes

add_executable(demo_wsg50 examples/demo_wsg50.cpp)
target_link_libraries(demo_wsg50 ${catkin_LIBRARIES} wsg50)

################################################################################################

# install targets

## executables and/or libraries for installation
install(
	TARGETS
		wsg50
		wsg50_controllers
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

## install header files
install(
	DIRECTORY
		include/wsg50/
	DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

## install ros_control plugin
install(FILES wsg50_controllers_plugin.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

## install models
install(DIRECTORY meshes DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY robots DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})