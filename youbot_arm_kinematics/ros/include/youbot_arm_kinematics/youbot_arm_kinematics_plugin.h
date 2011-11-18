/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* Author: Sachin Chitta
*********************************************************************/

#ifndef YOUBOT_ARM_KINEMATICS_PLUGIN_H
#define YOUBOT_ARM_KINEMATICS_PLUGIN_H

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <angles/angles.h>
#include <tf_conversions/tf_kdl.h>

#include <kinematics_msgs/GetPositionFK.h>
#include <kinematics_msgs/GetPositionIK.h>
#include <kinematics_msgs/GetKinematicSolverInfo.h>
#include <motion_planning_msgs/ArmNavigationErrorCodes.h>

#include <kdl/chainfksolverpos_recursive.hpp>

#include <boost/shared_ptr.hpp>

#include <kinematics_base/kinematics_base.h>

#include <youbot_arm_kinematics/forward_kinematics.h>
#include <youbot_arm_kinematics/inverse_kinematics_solver.h>

namespace youbot_arm_kinematics
{
class ArmKinematicsPlugin : public kinematics::KinematicsBase
{
	public:
		/** @class
		 *  @brief Plugin-able interface to the PR2 arm kinematics
		 */
		ArmKinematicsPlugin();

		/**
		 *  @brief Specifies if the node is active or not
		 *  @return True if the node is active, false otherwise.
		 */
		bool isActive();

		/**
		 * @brief Given a desired pose of the end-effector, compute the joint angles to reach it
		 * @param ik_link_name - the name of the link for which IK is being computed
		 * @param ik_pose the desired pose of the link
		 * @param ik_seed_state an initial guess solution for the inverse kinematics
		 * @return True if a valid solution was found, false otherwise
		 */
		bool getPositionIK(const geometry_msgs::Pose &ik_pose,
				const std::vector<double> &ik_seed_state,
				std::vector<double> &solution,
				int &error_code);

		/**
		 * @brief Given a desired pose of the end-effector, search for the joint angles required to reach it.
		 * This particular method is intended for "searching" for a solutions by stepping through the redundancy
		 * (or other numerical routines).
		 * @param ik_pose the desired pose of the link
		 * @param ik_seed_state an initial guess solution for the inverse kinematics
		 * @return True if a valid solution was found, false otherwise
		 */
		bool searchPositionIK(const geometry_msgs::Pose &ik_pose,
				const std::vector<double> &ik_seed_state,
				const double &timeout,
				std::vector<double> &solution,
				int &error_code);

		/**
		 * @brief Given a desired pose of the end-effector, search for the joint angles required to reach it.
		 * This particular method is intended for "searching" for a solutions by stepping through the redundancy
		 * (or other numerical routines).
		 * @param ik_pose the desired pose of the link
		 * @param ik_seed_state an initial guess solution for the inverse kinematics
		 * @return True if a valid solution was found, false otherwise
		 */
		bool searchPositionIK(const geometry_msgs::Pose &ik_pose,
				const std::vector<double> &ik_seed_state,
				const double &timeout,
				std::vector<double> &solution,
				const boost::function<void(const geometry_msgs::Pose &ik_pose,const std::vector<double> &ik_solution,int &error_code)> &desired_pose_callback,
				const boost::function<void(const geometry_msgs::Pose &ik_pose,const std::vector<double> &ik_solution,int &error_code)> &solution_callback,
				int &error_code);

		/**
		 * @brief Given a set of joint angles and a set of links, compute their pose
		 * @param request  - the request contains the joint angles, set of links for which poses are to be computed and a timeout
		 * @param response - the response contains stamped pose information for all the requested links
		 * @return True if a valid solution was found, false otherwise
		 */
		bool getPositionFK(const std::vector<std::string> &link_names,
			const std::vector<double> &joint_angles,
			std::vector<geometry_msgs::Pose> &poses);

		/**
		 * @brief  Initialization function for the kinematics
		 * @return True if initialization was successful, false otherwise
		 */
		bool initialize(std::string name);

		/**
		 * @brief  Return the frame in which the kinematics is operating
		 * @return the string name of the frame in which the kinematics is operating
		 */
		std::string getBaseFrame();

		/**
		 * @brief  Return the links for which kinematics can be computed
		 */
		std::string getToolFrame();

		/**
		 * @brief  Return all the joint names in the order they are used internally
		 */
		std::vector<std::string> getJointNames();

		/**
		 * @brief  Return all the link names in the order they are represented internally
		 */
		std::vector<std::string> getLinkNames();


    protected: // functions
		void desiredPoseCallback(const KDL::JntArray& jnt_array,
				const KDL::Frame& ik_pose,
				motion_planning_msgs::ArmNavigationErrorCodes& error_code);

		void jointSolutionCallback(const KDL::JntArray& jnt_array,
				const KDL::Frame& ik_pose,
				motion_planning_msgs::ArmNavigationErrorCodes& error_code);

    
    protected: // variables
		boost::shared_ptr<InverseKinematics>_ik;
		boost::shared_ptr<InverseKinematicsSolver> _ik_solver;
		boost::shared_ptr<ForwardKinematics> _fk_solver;
		kinematics_msgs::KinematicSolverInfo _ik_solver_info;
		kinematics_msgs::KinematicSolverInfo _fk_solver_info;
		ros::NodeHandle _node_handle;
		bool _active;
		int _dimension;
		std::string _root_name;
		boost::function<void(const geometry_msgs::Pose &ik_pose,const std::vector<double> &ik_solution,int &error_code)> _desiredPoseCallback;
		boost::function<void(const geometry_msgs::Pose &ik_pose,const std::vector<double> &ik_solution,int &error_code)> _solutionCallback;
};

}

#endif
