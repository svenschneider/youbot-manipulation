#ifndef FORWARD_KINEMATICS_H
#define FORWARD_KINEMATICS_H

// ROS includes
#include <kinematics_msgs/KinematicSolverInfo.h>
#include <kinematics_msgs/GetKinematicSolverInfo.h>
#include <kdl/chainfksolver.hpp>
#include <urdf/model.h>

/**
 * Interface for forward kinematic solvers for the youBot arm.
 */
class ForwardKinematics : public KDL::ChainFkSolverPos
{
	public:
		/**
		 * Dtor.
		 */
		virtual ~ForwardKinematics() {}


		/**
		 * Perform the FK calculation.
		 *
		 * @param q_in Input joint coordinates.
		 * @param p_out Reference to output cartesian pose.
		 * @return < 0 if something went wrong.
		 */
		virtual int JntToCart(const KDL::JntArray &q_in,
			KDL::Frame &p_out,
			int segmentNr = -1) = 0;

		/**
		 * @brief A method to get chain information about the serial chain that the FK operates on 
		 * @param info This class gets populated with information about the joints that FK operates on, including joint names and limits.
		 */
		virtual void getSolverInfo(kinematics_msgs::KinematicSolverInfo &info) const = 0;

		/**
		 * Get the index corresponding to the link name.
		 */
		virtual int getSegmentIndex(const std::string &name) const = 0;
};

#endif
