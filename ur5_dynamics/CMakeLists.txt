cmake_minimum_required(VERSION 3.4)
project(ur5_dynamics)

# C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# find catkin and any catkin packages
find_package(
	catkin REQUIRED
	COMPONENTS
		roscpp
		roslib
		urdf
		kdl_parser
		kdl_conversions
		eigen_conversions
		geometry_msgs
		ros_utils
		ur5_description
)

# Finding Eigen is somewhat complicated because of our need to support Ubuntu
# all the way back to saucy.  First we look for the Eigen3 cmake module
# provided by the libeigen3-dev on newer Ubuntu.  If that fails, then we
# fall-back to the version provided by cmake_modules, which is a stand-in.
find_package(Eigen3 QUIET)
if(NOT EIGEN3_FOUND)
	find_package(cmake_modules REQUIRED)
	find_package(Eigen REQUIRED)
	set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
endif()

# Note that eigen 3.2 (on Ubuntu Wily) only provides EIGEN3_INCLUDE_DIR,
# not EIGEN3_INCLUDE_DIRS, so we have to set the latter from the former.
if(NOT EIGEN3_INCLUDE_DIRS)
	set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()

# declare a catkin package
catkin_package(
	INCLUDE_DIRS
		include
		${EIGEN3_INCLUDE_DIR}
	LIBRARIES
		ur5_dynamics
	CATKIN_DEPENDS
		roscpp
		roslib
		urdf
		kdl_parser
		kdl_conversions
		eigen_conversions
		geometry_msgs
		ros_utils
		ur5_description
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

## ur5_dynamics
add_library(ur5_dynamics src/lib_ur5_dynamics.cpp)
target_link_libraries(ur5_dynamics ${catkin_LIBRARIES})

################################################################################################

# nodes

add_executable(demo_ur5_dynamics examples/demo_ur5_dynamics.cpp)
target_link_libraries(demo_ur5_dynamics ${catkin_LIBRARIES} ur5_dynamics)

# ...

################################################################################################

# install targets

## executables and/or libraries for installation
install(
	TARGETS
		ur5_dynamics
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

## install header files
install(
	DIRECTORY
		include/ur5_dynamics/
	DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)