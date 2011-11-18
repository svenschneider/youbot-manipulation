#include <youbot_arm_kinematics/arm_kdl_inverse_kinematics.h>

// ROS includes
#include <pr2_arm_kinematics/pr2_arm_kinematics_utils.h>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>

// Package includes
#include <youbot_arm_kinematics/solver_info_processor.h>


ArmKdlInverseKinematics::ArmKdlInverseKinematics(const urdf::Model &robot_model,
		const std::string &robot_description,
		const std::string &root_name,
		const std::string &tip_name)
{
	// create the KDL chain from the robot description
	if (!pr2_arm_kinematics::getKDLChain(robot_description, root_name, tip_name, _chain)) {
		ROS_ERROR("Could not load KDL tree");
		ROS_ASSERT(false);
	}

	SolverInfoProcessor solver_info_processor(robot_model, tip_name, root_name);
	_solver_info = solver_info_processor.getSolverInfo();

	for (unsigned int i = 0; i < _solver_info.joint_names.size(); i++) {
		_min_angles.push_back(_solver_info.limits[i].min_position);
		_max_angles.push_back(_solver_info.limits[i].max_position);
	}
}


ArmKdlInverseKinematics::~ArmKdlInverseKinematics()
{
}


int ArmKdlInverseKinematics::CartToJnt(const KDL::JntArray& q_init,
		const KDL::Frame& p_in,
		std::vector<KDL::JntArray>& q_out)
{
	KDL::JntArray q;
	int res = CartToJnt(q_init, p_in, q);

	if (res > 0) {
		q_out.clear();
		q_out.push_back(q);

		return 1;
	} else {
		q_out.clear();

		return -1;
	}
}


void ArmKdlInverseKinematics::getSolverInfo(kinematics_msgs::KinematicSolverInfo &info)
{
	info = _solver_info;
}


int ArmKdlInverseKinematics::CartToJnt(const KDL::JntArray& q_init,
		const KDL::Frame& p_in,
		KDL::JntArray& q_out)
{
	KDL::JntArray q_min(_min_angles.size());
	KDL::JntArray q_max(_max_angles.size());

	// copy the joint limits to a KDL array
	for (unsigned int i = 0; i < _min_angles.size(); i++) {
		q_min(i) = _min_angles[i];
		q_max(i) = _max_angles[i];
	}

	// Forward position solver
	KDL::ChainFkSolverPos_recursive fksolver1(_chain);
	// Inverse velocity solver
	KDL::ChainIkSolverVel_pinv iksolver1v(_chain);
	// Maximum 100 iterations, stop at accuracy 1e-6
	KDL::ChainIkSolverPos_NR_JL iksolverpos(_chain, q_min, q_max, fksolver1, iksolver1v, 1000, 1e-6);

	int ret = iksolverpos.CartToJnt(q_init, p_in, q_out);

	ROS_DEBUG("q_init: %f %f %f %f %f", q_init(0), q_init(1), q_init(2), q_init(3), q_init(4));
	ROS_DEBUG("q_out: %f %f %f %f %f", q_out(0), q_out(1), q_out(2), q_out(3), q_out(4));
	
	if (ret < 0) {
		ROS_DEBUG("Inverse Kinematic found no solution. KDL Return value = %i", ret);
		
		return -1;
	} else {
		ROS_DEBUG("Inverse Kinematic found a solution");
		
		return 1;
	}
}
