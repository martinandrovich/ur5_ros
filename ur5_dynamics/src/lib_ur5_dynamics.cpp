#include <ur5_dynamics/ur5_dynamics.h>
#include "ur5_inv.hpp"

#include <eigen_conversions/eigen_kdl.h>
#include <eigen_conversions/eigen_msg.h>
#include <kdl_conversions/kdl_msg.h>
#include <ros_utils/math.h>

bool
ur5_dynamics::init(const std::string& robot, const std::string& from, const std::string& to)
{
	if (is_init)
	{
		return true;
		ROS_WARN("KDL is already initialized.");
	}
	else
		ROS_WARN_STREAM("Initializing KDL for ur5_dynamics for links [" << from << ", " << to << "]...");

	if (not robot_model.initParam(robot))
	{
		ROS_ERROR_STREAM("Could not load URDF robot model from: " << robot);
		return false;
	}

	// compose KDL tree
	KDL::Tree kdl_tree;
	if (not kdl_parser::treeFromUrdfModel(robot_model, kdl_tree))
	{
		ROS_ERROR("Could not construct KDL tree from robot model.");
		return false;
	};

	// load KDL chain
	kdl_tree.getChain(from, to, kdl_chain);
	// ROS_INFO_STREAM("Using KDL chain from: " << from << " to: " << to);

	// initialize KDL solver(s)
	kdl_dyn_solver = new KDL::ChainDynParam(kdl_chain, KDL::Vector(0, 0, ur5::GRAVITY));
	kdl_jac_solver = new KDL::ChainJntToJacSolver(kdl_chain);
	kdl_jac_dot_solver = new KDL::ChainJntToJacDotSolver(kdl_chain);
	kdl_fk_solver = new KDL::ChainFkSolverPos_recursive(kdl_chain);

	// done
	ROS_INFO("Initialized KDL for ur5_dynamics.");
	is_init = true;

	return true;
}

void
ur5_dynamics::check_init()
{
	if (not is_init)
		ur5_dynamics::init();
}

Eigen::Vector6d
ur5_dynamics::gravity(const Eigen::Vector6d& q)
{
	ur5_dynamics::check_init();

	static auto q_kdl = KDL::JntArray(ur5::NUM_JOINTS);
	static auto g_kdl = KDL::JntArray(ur5::NUM_JOINTS);

	// load values of q from Eigen to JntArray
	for (auto i = 0; i < ur5::NUM_JOINTS; ++i)
		q_kdl(i) = q[i];

	// compute gravity
	kdl_dyn_solver->JntToGravity(q_kdl, g_kdl);

	// return as Eigen vector
	return g_kdl.data;
}

Eigen::Matrix6d
ur5_dynamics::mass(const Eigen::Vector6d& q)
{
	ur5_dynamics::check_init();

	static auto q_kdl = KDL::JntArray(ur5::NUM_JOINTS);
	static auto M_kdl = KDL::JntSpaceInertiaMatrix(ur5::NUM_JOINTS);

	// load values of q from Eigen to JntArray
	for (auto i = 0; i < ur5::NUM_JOINTS; ++i)
		q_kdl(i) = q[i];

	// compute gravity
	kdl_dyn_solver->JntToMass(q_kdl, M_kdl);

	// return as Eigen vector
	return M_kdl.data;
}

Eigen::Vector6d
ur5_dynamics::coriolis(const Eigen::Vector6d& q, const Eigen::Vector6d& qdot)
{
	ur5_dynamics::check_init();

	static auto q_kdl = KDL::JntArray(ur5::NUM_JOINTS);
	static auto qdot_kdl = KDL::JntArray(ur5::NUM_JOINTS);
	static auto c_kdl = KDL::JntArray(ur5::NUM_JOINTS);

	// load values of q and qdot from Eigen to JntArray
	for (auto i = 0; i < ur5::NUM_JOINTS; ++i)
	{
		q_kdl(i) = q[i];
		qdot_kdl(i) = qdot[i];
	}

	kdl_dyn_solver->JntToCoriolis(q_kdl, qdot_kdl, c_kdl);

	return c_kdl.data;
}

template<typename T>
T
ur5_dynamics::fwd_kin(const Eigen::Vector6d& q)
{
	// forward kinematics
	// computed with respect to end-effector link of robot, i.e. /end/ of link6 (ur5_ee)

	static_assert(std::is_same<T, Eigen::Isometry3d>::value || std::is_same<T, geometry_msgs::Pose>::value,
	              "Wrong type; use Eigen::Isometry3d or geometry_msgs::Pose.");

	ur5_dynamics::check_init();

	static auto frame_ee = KDL::Frame();
	static auto q_kdl = KDL::JntArray(ur5::NUM_JOINTS);

	// load values of q from Eigen to JntArray
	for (size_t i = 0; i < ur5::NUM_JOINTS; ++i)
		q_kdl(i) = q[i];

	kdl_fk_solver->JntToCart(q_kdl, frame_ee, -1);

	// return type according to template (Eigen or geometry_msgs)

	if constexpr (std::is_same<T, Eigen::Isometry3d>::value)
	{
		static Eigen::Isometry3d pose_ee;
		tf::transformKDLToEigen(frame_ee, pose_ee);
		return pose_ee;
	}

	if constexpr (std::is_same<T, geometry_msgs::Pose>::value)
	{
		static geometry_msgs::Pose pose_ee;
		tf::PoseKDLToMsg(frame_ee, pose_ee);
		return pose_ee;
	}
}

template<typename T>
std::vector<Eigen::Vector6d>
ur5_dynamics::inv_kin(const T& pose)
{
	// inverse kinematics: returns a vector of all 8 solutions
	// computed for the pose of the kinematic chain b_T_ee = 'base --> ee (flange)'

	static_assert(std::is_same<T, Eigen::Isometry3d>::value || std::is_same<T, geometry_msgs::Pose>::value,
	              "Wrong type; use Eigen::Isometry3d or geometry_msgs::Pose.");

	// convert pose to Eigen if necessary
	Eigen::Isometry3d b_T_ee;

	if constexpr (std::is_same<T, geometry_msgs::Pose>::value)
		tf::poseMsgToEigen(pose, b_T_ee);

	else
	if constexpr (std::is_same<T, Eigen::Isometry3d>::value)
		b_T_ee = pose;

	// orient axis (rotate 90 deg around Z axis)
	static const auto R_z = Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitZ());

	// compute inverse kinematics (multiple solutions)
	auto vec_q = ur5_inv_kin(b_T_ee * R_z);

	// check if solution is found
	if (vec_q.empty())
		ROS_ERROR_STREAM("ur5_dynamics::inv_kin() did not find any solutions!");

	// normalize joint angles to range [-pi, pi]
	for (auto& q : vec_q)
		for (auto i = 0; i < q.size(); i++)
			q[i] = math::normalize_angle(q[i]);
			// if (q[i] > M_PI) q[i] -= 2*M_PI; else if (q[i] > M_PI) q[i] += 2*M_PI;

	return vec_q;
}

template<typename T>
Eigen::Vector6d
ur5_dynamics::inv_kin(const T& pose, const Eigen::Vector6d& q)
{

	// inverse kinematics, returns the best Euclidean solution given initial robot configuration, q
	// computed with respect to end-effector link of robot, i.e. /end/ of link6 (ur5_ee)

	static_assert(std::is_same<T, Eigen::Isometry3d>::value || std::is_same<T, geometry_msgs::Pose>::value,
	              "Wrong type; use Eigen::Isometry3d or geometry_msgs::Pose.");

	// get vector of solutions
	const auto vec_q = ur5_dynamics::inv_kin(pose);

	auto q_best = vec_q[0];
	auto dist_best = std::numeric_limits<double>::max();

	// find the q[i] that has the least Euclidean distance wrt. given q
	for (const auto& qi : vec_q)
	{
		// compute distance; update if better than current best
		if (auto dist = (qi - q).array().pow(2).sum(); dist < dist_best)
		{
			q_best = qi;
			dist_best = dist;
		}
	}

	return q_best;
}

Eigen::Matrix6d
ur5_dynamics::jac(const Eigen::Vector6d& q)
{
	ur5_dynamics::check_init();

	static auto q_kdl = KDL::JntArray(ur5::NUM_JOINTS);
	static auto geo_jac = KDL::Jacobian(ur5::NUM_JOINTS);

	for (auto i = 0; i < ur5::NUM_JOINTS; ++i)
		q_kdl(i) = q[i];

	kdl_jac_solver->JntToJac(q_kdl, geo_jac);

	return geo_jac.data;
}

Eigen::Matrix6d
ur5_dynamics::jac_dot(const Eigen::Vector6d& q, const Eigen::Vector6d& qdot)
{
	ur5_dynamics::check_init();

	// load values of q and qdot from Eigen to JntArrayVel
	static KDL::JntArrayVel qdot_arr_kdl;
	for (auto i = 0; i < ur5::NUM_JOINTS; ++i)
	{
		qdot_arr_kdl.q(i) = q[i];
		qdot_arr_kdl.qdot(i) = qdot[i];
	}

	// determine jacobian dot (geometric)
	static auto geo_jac_dot = KDL::Jacobian(ur5::NUM_JOINTS);
	kdl_jac_dot_solver->JntToJacDot(qdot_arr_kdl, geo_jac_dot);

	return geo_jac_dot.data;
}

template<typename T>
Eigen::Matrix6d
ur5_dynamics::pinv_jac(const T& arg, double eps)
{

	// this is not the dynamical consistent pseudo inverse, but
	// it should for later usage be implemented, for now, this is fine.

	static_assert(std::is_same<T, Eigen::Matrix6d>::value || std::is_same<T, Eigen::Vector6d>::value,
	              "Wrong type use Matrix6d or Vector6d.");

	Eigen::Matrix6d jac;

	// Determine jacobian
	if constexpr (std::is_same<T, Eigen::Vector6d>::value)
		jac = ur5_dynamics::jac(arg);

	// Jacobian was an argument
	if constexpr (std::is_same<T, Eigen::Matrix6d>::value)
		jac = arg;

	// transpose
	Eigen::Matrix6d jac_T = jac.transpose();
	Eigen::Matrix6d jac_sym = jac_T * jac;

	// define SVD object
	Eigen::JacobiSVD<Eigen::Matrix6d> svd(jac_sym, Eigen::ComputeFullU | Eigen::ComputeFullV);

	// pseudo inverse
	Eigen::Matrix6d singular_inv = Eigen::Matrix6d::Zero();

	// singular_inv with dampening
	for (auto i = 0; i < jac.rows(); i++)
		singular_inv(i,i) = 1./(svd.singularValues()(i) + eps*eps);

	// calculate pinv
	const auto J_pinv = svd.matrixV() * singular_inv * svd.matrixU().transpose() * jac_T;

	return J_pinv;
}

template<typename T>
Eigen::Matrix6d ur5_dynamics::mani(const T& arg)
{

	static_assert(std::is_same<T, Eigen::Matrix6d>::value || std::is_same<T, Eigen::Vector6d>::value,
	              "Wrong type use Matrix6d or Vector6d.");

	Eigen::Matrix6d jac;

	// Determine jacobian
	if constexpr (std::is_same<T, Eigen::Vector6d>::value)
		jac = ur5_dynamics::jac(arg);

	// Jacobian was an argument
	if constexpr (std::is_same<T, Eigen::Matrix6d>::value)
		jac = arg;

	Eigen::Matrix6d man = jac*jac.transpose();

	// define SVD object
	Eigen::JacobiSVD<Eigen::Matrix6d> svd(man, Eigen::ComputeFullU | Eigen::ComputeFullV);

	// get eigen-vectors
	man.block<3, 3>(0, 0) << svd.matrixV().block<3, 3>(0, 0);

	// get eigen-values
	Eigen::Matrix3d singular = Eigen::Matrix3d::Zero();

	for (auto i = 0; i < singular.rows(); i++)
		singular(i, i) = svd.singularValues()(i);

	// construct manipulability matrix
	man.block<3, 3>(3, 3) << singular;

	return man;
}

// TEMPLATE SPECIALIZATIONS

template Eigen::Isometry3d ur5_dynamics::fwd_kin<Eigen::Isometry3d>(const Eigen::Vector6d& q);
template geometry_msgs::Pose ur5_dynamics::fwd_kin<geometry_msgs::Pose>(const Eigen::Vector6d& q);

template std::vector<Eigen::Vector6d> ur5_dynamics::inv_kin<Eigen::Isometry3d>(const Eigen::Isometry3d& pose);
template std::vector<Eigen::Vector6d> ur5_dynamics::inv_kin<geometry_msgs::Pose>(const geometry_msgs::Pose& pose);
template Eigen::Vector6d ur5_dynamics::inv_kin<Eigen::Isometry3d>(const Eigen::Isometry3d& pose, const Eigen::Vector6d& q);
template Eigen::Vector6d ur5_dynamics::inv_kin<geometry_msgs::Pose>(const geometry_msgs::Pose& pose, const Eigen::Vector6d& q);

template Eigen::Matrix6d ur5_dynamics::pinv_jac<Eigen::Matrix6d>(const Eigen::Matrix6d& jac, const double eps);
template Eigen::Matrix6d ur5_dynamics::pinv_jac<Eigen::Vector6d>(const Eigen::Vector6d& q, const double eps);

template Eigen::Matrix6d ur5_dynamics::mani<Eigen::Matrix6d>(const Eigen::Matrix6d& jac);
template Eigen::Matrix6d ur5_dynamics::mani<Eigen::Vector6d>(const Eigen::Vector6d& q);