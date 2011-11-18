//Software License Agreement (BSD License)

//Copyright (c) 2008, Willow Garage, Inc.
//All rights reserved.

//Redistribution and use in source and binary forms, with or without
//modification, are permitted provided that the following conditions
//are met:

// * Redistributions of source code must retain the above copyright
//   notice, this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above
//   copyright notice, this list of conditions and the following
//   disclaimer in the documentation and/or other materials provided
//   with the distribution.
// * Neither the name of Willow Garage, Inc. nor the names of its
//   contributors may be used to endorse or promote products derived
//   from this software without specific prior written permission.

//THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
//"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
//LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
//FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
//COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
//INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
//BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
//LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
//CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
//LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
//ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
//POSSIBILITY OF SUCH DAMAGE.


#ifndef ARM_KINEMATICS_CONSTRAINT_AWARE_UTILS_H
#define ARM_KINEMATICS_CONSTRAINT_AWARE_UTILS_H

// ROS
#include <ros/ros.h>
#include <urdf/model.h>

// System
#include <vector>
#include <Eigen/Array>

// KDL
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>

// TF
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_kdl.h>

// Kinematics
#include <kinematics_base/kinematics_base.h>

// ROS msgs
#include <kinematics_msgs/GetPositionFK.h>
#include <kinematics_msgs/GetPositionIK.h>
#include <kinematics_msgs/GetConstraintAwarePositionIK.h>
#include <kinematics_msgs/GetKinematicSolverInfo.h>

// MISC
#include <angles/angles.h>

using namespace angles;

namespace arm_kinematics_constraint_aware
{
  Eigen::Matrix4f KDLToEigenMatrix(const KDL::Frame &p);

  double computeEuclideanDistance(const std::vector<double> &array_1, 
                                  const KDL::JntArray &array_2);

  double distance(const urdf::Pose &transform);

  bool solveQuadratic(const double &a, 
                      const double &b, 
                      const double &c, 
                      double *x1, 
                      double *x2);

  Eigen::Matrix4f matrixInverse(const Eigen::Matrix4f &g);

  bool solveCosineEqn(const double &a, 
                      const double &b, 
                      const double &c, 
                      double &soln1, 
                      double &soln2);

  bool loadRobotModel(ros::NodeHandle node_handle, 
                      urdf::Model &robot_model, 
                      std::string &root_name, 
                      std::string &tip_name, 
                      std::string &xml_string);

  bool getKDLChain(const std::string &xml_string, 
                   const std::string &root_name, 
                   const std::string &tip_name, 
                   KDL::Chain &kdl_chain);

  bool getKDLTree(const std::string &xml_string, 
                   const std::string &root_name, 
                   const std::string &tip_name, 
                   KDL::Tree &kdl_chain);

  bool checkJointNames(const std::vector<std::string> &joint_names, 
                       const kinematics_msgs::KinematicSolverInfo &chain_info);

  bool checkLinkNames(const std::vector<std::string> &link_names,
                      const kinematics_msgs::KinematicSolverInfo &chain_info);

  bool checkLinkName(const std::string &link_name, 
                     const kinematics_msgs::KinematicSolverInfo &chain_info);
 
  bool checkRobotState(motion_planning_msgs::RobotState &robot_state,
                       const kinematics_msgs::KinematicSolverInfo &chain_info);

  bool checkFKService(kinematics_msgs::GetPositionFK::Request &request, 
                      kinematics_msgs::GetPositionFK::Response &response, 
                      const kinematics_msgs::KinematicSolverInfo &chain_info);
 
  bool checkIKService(kinematics_msgs::GetPositionIK::Request &request, 
                      kinematics_msgs::GetPositionIK::Response &response,
                      const kinematics_msgs::KinematicSolverInfo &chain_info);
 
  bool checkConstraintAwareIKService(kinematics_msgs::GetConstraintAwarePositionIK::Request &request, 
                                   kinematics_msgs::GetConstraintAwarePositionIK::Response &response,
                                   const kinematics_msgs::KinematicSolverInfo &chain_info);

  int getJointIndex(const std::string &name,
                    const kinematics_msgs::KinematicSolverInfo &chain_info);

  bool convertPoseToRootFrame(const geometry_msgs::PoseStamped &pose_msg, 
                              KDL::Frame &pose_kdl, 
                              const std::string &root_frame, 
                              const tf::TransformListener &tf);

  bool convertPoseToRootFrame(const geometry_msgs::PoseStamped &pose_msg, 
                              geometry_msgs::PoseStamped &pose_msg_out, 
                              const std::string &root_frame, 
                              const tf::TransformListener &tf);

  int getKDLSegmentIndex(const KDL::Chain &chain, 
                         const std::string &name);

  void getKDLChainInfo(const KDL::Chain &chain,
                       kinematics_msgs::KinematicSolverInfo &chain_info);

  void reorderJointState(sensor_msgs::JointState &joint_state, 
                         const kinematics_msgs::KinematicSolverInfo &chain_info);

  bool getChainInfoFromRobotModel(urdf::Model &robot_model,
                                  const std::string &root_name,
                                  const std::string &tip_name,
                                  kinematics_msgs::KinematicSolverInfo &chain_info);

  bool getChainInfo(const std::string &name, kinematics_msgs::KinematicSolverInfo &chain_info);

  motion_planning_msgs::ArmNavigationErrorCodes kinematicsErrorCodeToMotionPlanningErrorCode(const int &kinematics_error_code);

}

#endif// ARM_KINEMATICS_CONSTRAINT_AWARE_UTILS_H_
