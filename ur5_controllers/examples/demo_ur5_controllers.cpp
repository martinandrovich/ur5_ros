
#include <iostream>
#include <vector>

#include <ros/ros.h>
#include <ur5_controllers/ur5_controllers.h>

int main(int argc, char** argv)
{
	ros::init(argc, argv, "demo_ur5_controllers");
	ros::NodeHandle nh;
	
	// first launch the workcell and controller
	// roslaunch ur5_gazebo ur5.launch controller:="ur5_joint_position_controller"
	
	// execute single commnand (desired joint state)
	auto q_d = Eigen::Vector6d::Zero();
	ur5::command(q_d);
	
	// execute trajectory (blocks the thread)
	// std::vector<sensor_msgs::JointState> traj;	
	// ur5::exec_traj(traj);

	return 0;
}
