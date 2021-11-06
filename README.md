# ur5_ros
Integration of UR5 robot into ROS/Gazebo/MoveIt environment.

The meta-package consists of the following packages:

- [`ur5_description`](/ur5_description) - URDF descriptions of the UR5 robot and WSG gripper
- [`ur5_dynamics`](/ur5_dynamics) - Dynamics and kinematics of the UR5 robot
- [`ur5_controllers`](/ur5_controllers) - ROS Control controllers and interface for the UR5 robot
- [`ur5_gazebo`](/ur5_gazebo) - Gazebo interface for the UR5 robot
- [`ur5_moveit_config`](/ur5_moveit_config) - MoveIt! configuration for the UR5 robot
- [`ur5_planner`](/ur5_gripper) - Motion planning and reachability analysis using MoveIt for the UR5 robot
- [`ur5_msgs`](/ur5_msgs) - ROS messages/services for the UR5 robots
