# wsg50

Integration of WSG50 gripper into ROS, including a (fake) JointEffortController and command interface.

The file `wsg50.launch` launches the `ros_control` controller and sets the `ee_T_tcp` ROS parameter (such that it can be used by e.g. `ur5_description`).

An example use-case of the WSG50 gripper with the UR5 robot can be seen by examining the `ur5.launch` file of the `ur5_gazebo` package. 