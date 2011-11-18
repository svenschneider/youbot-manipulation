#ifndef SOLVER_INFO_PROCESSOR_H
#define SOLVER_INFO_PROCESSOR_H

#include <boost/shared_ptr.hpp>
#include <kinematics_msgs/KinematicSolverInfo.h>
#include <urdf/model.h>


class SolverInfoProcessor
{
	public:
		/**
		 * Ctor.
		 */
		SolverInfoProcessor(const urdf::Model &robot_model,
			const std::string &tip_name,
			const std::string &root_name);

		/**
		 * Dtor.
		 */
		virtual ~SolverInfoProcessor();

		/**
		 * Get the extracted information.
		 */
		kinematics_msgs::KinematicSolverInfo getSolverInfo() const;


	private:
		/**
		 * Add a joint that is described by URDF to a kinematic chain description.
		 * @param joint The joint in URDF description.
		 * @param info The information about the IK solver which will be extended.
		 */
		void addJointToChainInfo(boost::shared_ptr<const urdf::Joint> joint, kinematics_msgs::KinematicSolverInfo &info);


	private:
		/**
		 * The extracted solver information.
		 */
		kinematics_msgs::KinematicSolverInfo _solver_info;

};

#endif

