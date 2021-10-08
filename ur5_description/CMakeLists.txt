cmake_minimum_required(VERSION 3.4)
project(ur5_description)

# find catkin and any catkin packages
find_package(
	catkin REQUIRED
	COMPONENTS
		xacro
)

# declare a catkin package
catkin_package(
	CATKIN_DEPENDS
		xacro
)

# additional locations of header files
include_directories(
	${catkin_INCLUDE_DIRS}
)

# install models
install(DIRECTORY meshes DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY robots DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})