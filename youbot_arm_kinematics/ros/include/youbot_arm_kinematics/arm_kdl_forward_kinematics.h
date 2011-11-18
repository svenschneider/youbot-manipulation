#ifndef ARM_KDL_FORWARD_KINEMATICS_H
#define ARM_KDL_FORWARD_KINEMATICS_H

// Package includes
#include <youbot_arm_kinematics/forward_kinematics.h>

/**
 * A FK solver for the youBot arm using KDL.
 */
class ArmKdlForwardKinematics : public ForwardKinematics
{
	public:
		/**
		 * Ctor.
		 *
		 * @brief Initialize the solver by providing a urdf::Model and a root and tip name.
		 * @param robot_model A urdf::Model representation of the youBot robot model
		 * @param robot_description The XML string that of a urdf::Model which represents the youBot robot model
		 * @param root_name The root joint name of the arm 
		 * @param joint_name The tip joint name of the arm
		 */
		ArmKdlForwardKinematics(const urdf::Model &robot_model,
			const std::string &robot_description,
			const std::string &root_name,
			const std::string &tip_name);

		/**
		 * Dtor.
		 */
		virtual ~ArmKdlForwardKinematics();

		/**
		 * @see ForwardKinematics::JntToCart
		 */
		int JntToCart(const KDL::JntArray &q_in,
			KDL::Frame &p_out,
			int segmentNr = -1);

		/**
		 * @see ForwardKinematics::getSolverInfo
		 */
		void getSolverInfo(kinematics_msgs::KinematicSolverInfo &response) const;

		/**
		 * @see ForwardKinematics::getSegmentIndex
		 */
		int getSegmentIndex(const std::string &name) const;


	private:
		/**
		 * Add a joint that is described by URDF to a kinematic chain description.
		 * @param joint The joint in URDF description.
		 * @param info The information about the IK solver which will be extended.
		 */
		void addJointToChainInfo(boost::shared_ptr<const urdf::Joint> joint, kinematics_msgs::KinematicSolverInfo &info);


	private:
		/**
		 * The chain that the inverse kinematics is solved for.
		 */
		KDL::Chain _chain;

		/**
		 * Minimum joint limits.
		 */
		std::vector<double> _min_angles;

		/**
		 * Maximum joint limits.
		 */
		std::vector<double> _max_angles;

		/**
		 * Information about the IK solver.
		 */
		kinematics_msgs::KinematicSolverInfo _solver_info;
};

#endif
