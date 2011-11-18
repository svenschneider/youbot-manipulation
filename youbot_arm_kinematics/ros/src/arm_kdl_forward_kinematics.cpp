#include <youbot_arm_kinematics/arm_kdl_forward_kinematics.h>

// ROS includes
#include <pr2_arm_kinematics/pr2_arm_kinematics_utils.h>
#include <kdl/chainfksolverpos_recursive.hpp>

// Package includes
#include <youbot_arm_kinematics/solver_info_processor.h>


ArmKdlForwardKinematics::ArmKdlForwardKinematics(const urdf::Model &robot_model,
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
	for (unsigned int i = 0; i < _solver_info.joint_names.size(); i++) {
		_solver_info.joint_names = solver_info_processor.getSolverInfo().joint_names;
	}
}


ArmKdlForwardKinematics::~ArmKdlForwardKinematics()
{
}


int ArmKdlForwardKinematics::JntToCart(const KDL::JntArray &q_in,
		KDL::Frame &p_out,
		int segmentNr)
{
	KDL::ChainFkSolverPos_recursive fksolver(_chain);

	return fksolver.JntToCart(q_in, p_out, segmentNr);
}


void ArmKdlForwardKinematics::getSolverInfo(kinematics_msgs::KinematicSolverInfo &info) const
{
	info = _solver_info;
}


int ArmKdlForwardKinematics::getSegmentIndex(const std::string &name) const
{
	return pr2_arm_kinematics::getKDLSegmentIndex(_chain, name);
}
