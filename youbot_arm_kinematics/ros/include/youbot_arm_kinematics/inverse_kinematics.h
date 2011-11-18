#ifndef INVERSE_KINEMATICS_H
#define INVERSE_KINEMATICS_H

// ROS includes
#include <kinematics_msgs/KinematicSolverInfo.h>
#include <kinematics_msgs/GetKinematicSolverInfo.h>
#include <kdl/chainiksolver.hpp>
#include <urdf/model.h>

/**
 * Interface for inverse kinematic solvers for the youBot arm.
 * Implementations of the interface can e.g. be numerical or analytical IK solvers.
 */
class InverseKinematics
{
	public:
		/**
		 * Dtor.
		 */
		virtual ~InverseKinematics() {}

		/**
		 * Perform the IK calculation.
		 *
		 * @param q_init The initial guess for the inverse kinematics solution.
		 * @param p_in A KDL::Frame representation of the position of the end-effector for which the IK is being solved.
		 * @param q_out A vector of inverse kinematic solutions (if it exists).
		 * @return < 0 if no solution is found
		 */
		virtual int CartToJnt(const KDL::JntArray& q_init, 
			const KDL::Frame& p_in, 
			std::vector<KDL::JntArray>& q_out) = 0;

		/**
		 * @brief A method to get chain information about the serial chain that the IK operates on 
		 * @param info This class gets populated with information about the joints that IK operates on, including joint names and limits.
		 */
		virtual void getSolverInfo(kinematics_msgs::KinematicSolverInfo &info) = 0;
};

#endif
