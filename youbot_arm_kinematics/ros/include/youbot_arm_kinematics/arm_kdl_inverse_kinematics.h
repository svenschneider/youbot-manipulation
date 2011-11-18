#ifndef ARM_KDL_INVERSE_KINEMATICS_H
#define ARM_KDL_INVERSE_KINEMATICS_H

// Package includes
#include <youbot_arm_kinematics/inverse_kinematics.h>

/**
 * An IK solver for the youBot arm using KDL.
 */
class ArmKdlInverseKinematics : public InverseKinematics
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
		ArmKdlInverseKinematics(const urdf::Model &robot_model,
			const std::string &robot_description,
			const std::string &root_name,
			const std::string &tip_name);

		/**
		 * Dtor.
		 */
		virtual ~ArmKdlInverseKinematics();

		/**
		 * @see InverseKinematics::CartToJnt
		 */
		int CartToJnt(const KDL::JntArray& q_init,
			const KDL::Frame& p_in,
			std::vector<KDL::JntArray>& q_out);

		/**
		 * @see InverseKinematics::getSolverInfo
		 */
		void getSolverInfo(kinematics_msgs::KinematicSolverInfo &response);


	private: //functions
		/**
		 * Solve the inverse kinematics and return one set of joint angles.
		 *
		 * @param q_init The initial guess for the inverse kinematics solution.
		 * @param p_in A KDL::Frame representation of the position of the end-effector for which the IK is being solved.
		 * @param q_out A single inverse kinematic solution (if it exists).
		 * @return < 0 if no solution is found
		 */
		int CartToJnt(const KDL::JntArray& q_init,
			const KDL::Frame& p_in,
			KDL::JntArray& q_out);


	private: // variables
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
