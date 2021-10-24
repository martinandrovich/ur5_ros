
#include <ur5_controllers/interface.h>
#include <ros/ros.h>

int main(int argc, char** argv)
{
	ros::init(argc, argv, "demo_ur5_controllers");
	ros::NodeHandle nh;
	
	// first launch the workcell and controller
	// roslaunch ur5_gazebo ur5.launch controller:="ur5_joint_position_controller"
	
	// construct joint trajectory
	std::vector<sensor_msgs::JointState> traj;
	
	// execute trajectory (blocks the thread)
	ur5::exec_traj(traj);

	return 0;
}
