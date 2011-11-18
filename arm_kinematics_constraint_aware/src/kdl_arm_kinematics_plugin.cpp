/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * Author: Sachin Chitta, David Lu!!, Ugo Cupcic
 */

#include <arm_kinematics_constraint_aware/kdl_arm_kinematics_plugin.h>
#include <pluginlib/class_list_macros.h>

using namespace KDL;
using namespace tf;
using namespace std;
using namespace ros;

//register KDLArmKinematics as a KinematicsBase implementation
PLUGINLIB_DECLARE_CLASS(arm_kinematics_constraint_aware,KDLArmKinematicsPlugin, arm_kinematics_constraint_aware::KDLArmKinematicsPlugin, kinematics::KinematicsBase)

namespace arm_kinematics_constraint_aware {

KDLArmKinematicsPlugin::KDLArmKinematicsPlugin():active_(false)
{
  srand ( time(NULL) );
}

bool KDLArmKinematicsPlugin::isActive()
{
  if(active_)
    return true;
  return false;
}

double KDLArmKinematicsPlugin::genRandomNumber(const double &min, const double &max)
{
  int rand_num = rand()%100+1;
  double result = min + (double)((max-min)*rand_num)/101.0;
  return result;
}

KDL::JntArray KDLArmKinematicsPlugin::getRandomConfiguration()
{
  KDL::JntArray jnt_array;
  jnt_array.resize(dimension_);
  for(unsigned int i=0; i < dimension_; i++)
    jnt_array(i) = genRandomNumber(joint_min_(i),joint_max_(i));
  return jnt_array;
}

bool KDLArmKinematicsPlugin::initialize(std::string name)
{
  // Get URDF XML
  std::string urdf_xml, full_urdf_xml;
  ros::NodeHandle node_handle;
  ros::NodeHandle private_handle("~"+name);
  node_handle.param("urdf_xml",urdf_xml,std::string("robot_description"));
  node_handle.searchParam(urdf_xml,full_urdf_xml);
  ROS_DEBUG("Reading xml file from parameter server");
  std::string result;
  if (!node_handle.getParam(full_urdf_xml, result)) {
    ROS_FATAL("Could not load the xml from parameter server: %s", urdf_xml.c_str());
    return false;
  }

  // Get Root and Tip From Parameter Service
  if (!private_handle.getParam("root_name", root_name_)) {
    ROS_FATAL("GenericIK: No root name found on parameter server");
    return false;
  }
  if (!private_handle.getParam("tip_name", tip_name_)) {
    ROS_FATAL("GenericIK: No tip name found on parameter server");
    return false;
  }
  // Load and Read Models
  if (!loadModel(result)) {
    ROS_FATAL("Could not load models!");
    return false;
  }

  // Get Solver Parameters
  int max_iterations;
  double epsilon;

  private_handle.param("max_solver_iterations", max_iterations, 1000);
  private_handle.param("max_search_iterations", max_search_iterations_, 1000);
  private_handle.param("epsilon", epsilon, 1e-2);

  // Build Solvers
  fk_solver_.reset(new KDL::ChainFkSolverPos_recursive(kdl_chain_));
  ik_solver_vel_.reset(new KDL::ChainIkSolverVel_pinv(kdl_chain_));
  ik_solver_pos_.reset(new KDL::ChainIkSolverPos_NR_JL(kdl_chain_, joint_min_, joint_max_,*fk_solver_, *ik_solver_vel_, max_iterations, epsilon));
  active_ = true;
  return true;
}

bool KDLArmKinematicsPlugin::loadModel(const std::string xml) 
{
  urdf::Model robot_model;
  KDL::Tree tree;

  if (!robot_model.initString(xml)) {
    ROS_FATAL("Could not initialize robot model");
    return -1;
  }
  if (!kdl_parser::treeFromString(xml, tree)) {
    ROS_ERROR("Could not initialize tree object");
    return false;
  }
  if (!tree.getChain(root_name_, tip_name_, kdl_chain_)) {
    ROS_ERROR("Could not initialize chain object");
    return false;
  }
  if (!readJoints(robot_model)) {
    ROS_FATAL("Could not read information about the joints");
    return false;
  }
  return true;
}

bool KDLArmKinematicsPlugin::readJoints(urdf::Model &robot_model) 
{
  dimension_ = 0;
  // get joint maxs and mins
  boost::shared_ptr<const urdf::Link> link = robot_model.getLink(tip_name_);
  boost::shared_ptr<const urdf::Joint> joint;
  while (link && link->name != root_name_) {
    joint = robot_model.getJoint(link->parent_joint->name);
    if (!joint) {
      ROS_ERROR("Could not find joint: %s",link->parent_joint->name.c_str());
      return false;
    }
    if (joint->type != urdf::Joint::UNKNOWN && joint->type != urdf::Joint::FIXED) {
      ROS_INFO( "adding joint: [%s]", joint->name.c_str() );
      dimension_++;
    }
    link = robot_model.getLink(link->getParent()->name);
  }
  joint_min_.resize(dimension_);
  joint_max_.resize(dimension_);
  chain_info_.joint_names.resize(dimension_);
  chain_info_.limits.resize(dimension_);
  link = robot_model.getLink(tip_name_);
  if(link)
    chain_info_.link_names.push_back(tip_name_);

  unsigned int i = 0;
  while (link && i < dimension_) {
    joint = robot_model.getJoint(link->parent_joint->name);
    if (joint->type != urdf::Joint::UNKNOWN && joint->type != urdf::Joint::FIXED) {
      ROS_INFO( "getting bounds for joint: [%s]", joint->name.c_str() );

      float lower, upper;
      int hasLimits;
      if ( joint->type != urdf::Joint::CONTINUOUS ) {
        lower = joint->limits->lower;
        upper = joint->limits->upper;
        hasLimits = 1;
      } else {
        lower = -M_PI;
        upper = M_PI;
        hasLimits = 0;
      }
      int index = dimension_ - i -1;

      joint_min_.data[index] = lower;
      joint_max_.data[index] = upper;
      chain_info_.joint_names[index] = joint->name;
      chain_info_.limits[index].joint_name = joint->name;
      chain_info_.limits[index].has_position_limits = hasLimits;
      chain_info_.limits[index].min_position = lower;
      chain_info_.limits[index].max_position = upper;
      i++;
    }
    link = robot_model.getLink(link->getParent()->name);
  }
  return true;
}

int KDLArmKinematicsPlugin::getJointIndex(const std::string &name) 
{
  for (unsigned int i=0; i < chain_info_.joint_names.size(); i++) {
    if (chain_info_.joint_names[i] == name)
      return i;
  }
  return -1;
}

int KDLArmKinematicsPlugin::getKDLSegmentIndex(const std::string &name) 
{
  int i=0; 
  while (i < (int)kdl_chain_.getNrOfSegments()) {
    if (kdl_chain_.getSegment(i).getName() == name) {
      return i+1;
    }
    i++;
  }
  return -1;
}

bool KDLArmKinematicsPlugin::getPositionIK(const geometry_msgs::Pose &ik_pose,
                                           const std::vector<double> &ik_seed_state,
                                           std::vector<double> &solution,
                                           int &error_code)
{
  if(!active_)
  {
    ROS_ERROR("kinematics not active");
    return false;
  }
  KDL::Frame pose_desired;
  tf::PoseMsgToKDL(ik_pose, pose_desired);
  //Do the inverse kinematics
  KDL::JntArray jnt_pos_in;
  KDL::JntArray jnt_pos_out;
  jnt_pos_in.resize(dimension_);
  for(unsigned int i=0; i < dimension_; i++)
  {
    jnt_pos_in(i) = ik_seed_state[i];
  }
  int ik_valid = ik_solver_pos_->CartToJnt(jnt_pos_in,pose_desired,jnt_pos_out);
  if(ik_valid >= 0)
  {
    solution.resize(dimension_);
    for(unsigned int i=0; i < dimension_; i++)
    {
      solution[i] = jnt_pos_out(i);
    }
    error_code = kinematics::SUCCESS;
    return true;
  }
  else
  {
    ROS_DEBUG("An IK solution could not be found");   
    error_code = kinematics::NO_IK_SOLUTION;
    return false;
  }
}

bool KDLArmKinematicsPlugin::searchPositionIK(const geometry_msgs::Pose &ik_pose,
                                              const std::vector<double> &ik_seed_state,
                                              const double &timeout,
                                              std::vector<double> &solution,
                                              int &error_code)
{
  if(!active_)
  {
    ROS_ERROR("kinematics not active");
    error_code = kinematics::INACTIVE;
    return false;
  }
  KDL::Frame pose_desired;
  tf::PoseMsgToKDL(ik_pose, pose_desired);

  //Do the IK
  KDL::JntArray jnt_pos_in;
  KDL::JntArray jnt_pos_out;
  jnt_pos_in.resize(dimension_);
  for(unsigned int i=0; i < dimension_; i++)
  {
    jnt_pos_in(i) = ik_seed_state[i];
  }

  int ik_valid = ik_solver_pos_->CartToJnt(jnt_pos_in,pose_desired,jnt_pos_out);

  if(ik_valid >= 0)
  {
    solution.resize(dimension_);
    for(unsigned int i=0; i < dimension_; i++)
      solution[i] = jnt_pos_out(i);
    error_code = kinematics::SUCCESS;
    return true;
  }
  else
  {
    ROS_DEBUG("An IK solution could not be found");   
    error_code = kinematics::NO_IK_SOLUTION;
    return false;
  }
}

bool KDLArmKinematicsPlugin::searchPositionIK(const geometry_msgs::Pose &ik_pose,
                                              const std::vector<double> &ik_seed_state,
                                              const double &timeout,
                                              std::vector<double> &solution,
                                              const boost::function<void(const geometry_msgs::Pose &ik_pose,const std::vector<double> &ik_solution,int &error_code)> &desired_pose_callback,
                                              const boost::function<void(const geometry_msgs::Pose &ik_pose,const std::vector<double> &ik_solution,int &error_code)> &solution_callback,
                                              int &error_code)  
{
  if(!active_)
  {
    ROS_ERROR("kinematics not active");
    error_code = kinematics::INACTIVE;
    return false;
  }
  KDL::Frame pose_desired;
  tf::PoseMsgToKDL(ik_pose, pose_desired);

  //Do the IK
  KDL::JntArray jnt_pos_in;
  KDL::JntArray jnt_pos_out;
  jnt_pos_in.resize(dimension_);
  for(unsigned int i=0; i < dimension_; i++)
    jnt_pos_in(i) = ik_seed_state[i];
 
  if(!desired_pose_callback.empty())
    desired_pose_callback(ik_pose,ik_seed_state,error_code);
  
  if(error_code < 0)
  {
    ROS_ERROR("Could not find inverse kinematics for desired end-effector pose since the pose may be in collision");
    return false;
  }
  for(int i=0; i < max_search_iterations_; i++)
  {
    jnt_pos_in = getRandomConfiguration();
    int ik_valid = ik_solver_pos_->CartToJnt(jnt_pos_in,pose_desired,jnt_pos_out);                      
    if(ik_valid < 0)                                                                                                       
      continue;
    std::vector<double> solution_local;
    solution_local.resize(dimension_);
    for(unsigned int j=0; j < dimension_; j++)
      solution_local[j] = jnt_pos_out(j);
    solution_callback(ik_pose,solution_local,error_code);
    if(error_code == kinematics::SUCCESS)
    {
      solution = solution_local;
      return true;
    }
  }
  ROS_DEBUG("An IK that satisifes the constraints and is collision free could not be found");   
  if (error_code == 0)
    error_code = kinematics::NO_IK_SOLUTION;
  return false;
}

bool KDLArmKinematicsPlugin::getPositionFK(const std::vector<std::string> &link_names,
                                           const std::vector<double> &joint_angles,
                                           std::vector<geometry_msgs::Pose> &poses)
{
  if(!active_)
  {
    ROS_ERROR("kinematics not active");
    return false;
  }
  
  KDL::Frame p_out;
  KDL::JntArray jnt_pos_in;
  geometry_msgs::PoseStamped pose;
  tf::Stamped<tf::Pose> tf_pose;
  
  jnt_pos_in.resize(dimension_);
  for(unsigned int i=0; i < dimension_; i++)
  {
    jnt_pos_in(i) = joint_angles[i];
  }
  
  poses.resize(link_names.size());
  
  bool valid = true;
  for(unsigned int i=0; i < poses.size(); i++)
  {
    ROS_DEBUG("End effector index: %d",getKDLSegmentIndex(link_names[i]));
    if(fk_solver_->JntToCart(jnt_pos_in,p_out,getKDLSegmentIndex(link_names[i])) >=0)
    {
      tf::PoseKDLToMsg(p_out,poses[i]);
    }
    else
    {
      ROS_ERROR("Could not compute FK for %s",link_names[i].c_str());
      valid = false;
    }
  }
  return valid;
}

std::string KDLArmKinematicsPlugin::getBaseFrame()
{
  if(!active_)
  {
    ROS_ERROR("kinematics not active");
    return std::string("");
  }
  return root_name_;
}

std::string KDLArmKinematicsPlugin::getToolFrame()
{
  if(!active_ || chain_info_.link_names.empty())
  {
    ROS_ERROR("kinematics not active");
    return std::string("");
  }
  return chain_info_.link_names[0];
}

std::vector<std::string> KDLArmKinematicsPlugin::getJointNames()
{
  if(!active_)
  {
    std::vector<std::string> empty;
    ROS_ERROR("kinematics not active");
    return empty;
  }
  return chain_info_.joint_names;
}

std::vector<std::string> KDLArmKinematicsPlugin::getLinkNames()
{
  if(!active_)
  {
    std::vector<std::string> empty;
    ROS_ERROR("kinematics not active");
    return empty;
  }
  return chain_info_.link_names;
}

} // namespace
