#pragma once

#include <string>
#include <Eigen/Eigen>

#include <ros/ros.h>
#include <urdf/model.h>

#include <kdl_parser/kdl_parser.hpp>
#include <kdl/tree.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainjnttojacdotsolver.hpp>
#include <kdl/chainjnttojacsolver.hpp>

#include <geometry_msgs/Pose.h>
#include <ur5_description/ur5.h>


class ur5_dynamics
{

public:

	static bool
	init(
		const std::string& robot = ur5::ROBOT_DESCRIPTION,
		const std::string& from  = ur5::LINKS.URDF("first"),
		const std::string& to    = ur5::LINKS.URDF("last")
	);

	static Eigen::Vector6d
	gravity(const Eigen::Vector6d& q);

	static Eigen::Matrix6d
	mass(const Eigen::Vector6d& q);

	static Eigen::Vector6d
	coriolis(const Eigen::Vector6d& q, const Eigen::Vector6d& qdot);

	template<typename T = Eigen::Matrix4d>
	static T
	fwd_kin(const Eigen::Vector6d& q);

	template<typename T = Eigen::Matrix4d>
	static Eigen::Vector6d
	inv_kin(const T& frame, const Eigen::Vector6d& q);

	template<typename T = Eigen::Matrix4d>
	static Eigen::MatrixXd
	inv_kin(const T& frame);

	template<typename T = Eigen::Vector6d>
	static Eigen::Matrix6d
	pinv_jac(const T& arg, const double eps = 1.0e-5);

	template<typename T = Eigen::Vector6d>
	static Eigen::Matrix6d
	mani(const T& arg);

	static Eigen::Matrix6d
	jac(const Eigen::Vector6d& q);

	static Eigen::Matrix6d
	jac_dot(const Eigen::Vector6d& q, const Eigen::Vector6d& qdot);

private:

	static void
	check_init();

	static inline bool                             is_init = false;
	static inline urdf::Model                      robot_model;

	static inline KDL::Chain                       kdl_chain;
	static inline KDL::ChainDynParam*              kdl_dyn_solver;
	static inline KDL::ChainJntToJacSolver*        kdl_jac_solver;
	static inline KDL::ChainJntToJacDotSolver*     kdl_jac_dot_solver;
	static inline KDL::ChainFkSolverPos_recursive* kdl_fk_solver;
};
