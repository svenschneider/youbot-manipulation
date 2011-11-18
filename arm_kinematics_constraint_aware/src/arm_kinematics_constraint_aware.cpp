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
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
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
 * Author: Sachin Chitta
 */

#include <arm_kinematics_constraint_aware/arm_kinematics_constraint_aware.h>

#include <sensor_msgs/JointState.h>
#include <kinematics_msgs/utils.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

namespace arm_kinematics_constraint_aware {

static const std::string IK_WITH_COLLISION_SERVICE = "get_constraint_aware_ik";
  static const std::string IK_INFO_SERVICE = "get_ik_solver_info";
  static const std::string FK_INFO_SERVICE = "get_fk_solver_info";
static const std::string IK_SERVICE = "get_ik";
static const std::string FK_SERVICE = "get_fk";
static const double IK_DEFAULT_TIMEOUT = 10.0;

ArmKinematicsConstraintAware::ArmKinematicsConstraintAware(): kinematics_loader_("kinematics_base","kinematics::KinematicsBase"),node_handle_("~"),setup_collision_environment_(false)
{
  std::string group_name, kinematics_solver_name;
  node_handle_.param<bool>("visualize_solution",visualize_solution_,true);
  node_handle_.param<std::string>("group", group_, std::string());
  node_handle_.param<std::string>("kinematics_solver",kinematics_solver_name," ");
  ROS_INFO("Using kinematics solver name: %s",kinematics_solver_name.c_str());
  if (group_.empty())
  {
    ROS_ERROR("No 'group' parameter specified. Without the name of the group of joints to monitor, node cannot compute collision aware inverse kinematics");
    active_ = false;
    return;
  }

  kinematics_solver_ = NULL;
  try
  {
    kinematics_solver_ = kinematics_loader_.createClassInstance(kinematics_solver_name);
  }
  catch(pluginlib::PluginlibException& ex)
  {
    ROS_ERROR("The plugin failed to load. Error: %s", ex.what());    //handle the class failing to load
    active_ = false;
    return;
  }
  if(kinematics_solver_->initialize(group_))
    active_ = true;
  else
  {
    active_ = false;
    return;
  }
  root_name_ = kinematics_solver_->getBaseFrame();
  if(!getChainInfo(group_,chain_info_))
  {
    ROS_ERROR("Could not construct chain info for group %s",group_.c_str());
    return;
  }
  advertiseBaseKinematicsServices();
  if(setupCollisionEnvironment())
  {
    ROS_DEBUG("Advertising constraint aware services");
    advertiseConstraintIKService();
    ROS_INFO("Collision environment setup.");
  }
  else
  {
    ROS_ERROR("Could not initialize collision environment");
  }
}

void ArmKinematicsConstraintAware::advertiseBaseKinematicsServices()
{
  ik_service_ = node_handle_.advertiseService(IK_SERVICE,&ArmKinematicsConstraintAware::getPositionIK,this);
  fk_service_ = node_handle_.advertiseService(FK_SERVICE,&ArmKinematicsConstraintAware::getPositionFK,this);
  ik_solver_info_service_ = node_handle_.advertiseService(IK_INFO_SERVICE,&ArmKinematicsConstraintAware::getIKSolverInfo,this);
  fk_solver_info_service_ = node_handle_.advertiseService(FK_INFO_SERVICE,&ArmKinematicsConstraintAware::getFKSolverInfo,this);
}

void ArmKinematicsConstraintAware::advertiseConstraintIKService()
{
  ik_collision_service_ = node_handle_.advertiseService(IK_WITH_COLLISION_SERVICE,&ArmKinematicsConstraintAware::getConstraintAwarePositionIK,this);
  display_trajectory_publisher_ = root_handle_.advertise<motion_planning_msgs::DisplayTrajectory>("ik_solution_display", 1);
}

bool ArmKinematicsConstraintAware::isReady(motion_planning_msgs::ArmNavigationErrorCodes &error_code)
{
 if(!active_)
  {
    ROS_ERROR("IK service is not ready");
    return false;
  }
  if(!setup_collision_environment_)
  {
    ROS_INFO("Waiting for collision environment setup.");
    if(!setupCollisionEnvironment())
    {
      ROS_INFO("Could not initialize collision environment");
      error_code.val = error_code.COLLISION_CHECKING_UNAVAILABLE;
      return false;
    }    
    else
    {
      setup_collision_environment_ = true;
    }
  }
  error_code.val = error_code.SUCCESS;
  return true;
}

bool ArmKinematicsConstraintAware::getConstraintAwarePositionIK(kinematics_msgs::GetConstraintAwarePositionIK::Request &request_in,
                                                                kinematics_msgs::GetConstraintAwarePositionIK::Response &response)
{
  if(!isReady(response.error_code))
    return true;

  if(!checkConstraintAwareIKService(request_in,response,chain_info_))
  {
    ROS_ERROR("IK service request is malformed");
    return true;
  }

  ros::Time start_time = ros::Time::now();
  ROS_INFO("Received IK request is in the frame: %s",request_in.ik_request.pose_stamped.header.frame_id.c_str());

  ik_request_ = request_in.ik_request;
  collision_operations_ = request_in.ordered_collision_operations;
  link_padding_ = request_in.link_padding;
  allowed_contacts_ = request_in.allowed_contacts;
  constraints_ = request_in.constraints;

  geometry_msgs::PoseStamped pose_msg_in = ik_request_.pose_stamped;
  geometry_msgs::PoseStamped pose_msg_out;
  if(!arm_kinematics_constraint_aware::convertPoseToRootFrame(pose_msg_in,pose_msg_out,root_name_,tf_))
  {
    response.error_code.val = response.error_code.FRAME_TRANSFORM_FAILURE;
    return true;
  }
  motion_planning_msgs::Constraints emp;

  //setting up planning monitor
  planning_monitor_->prepareForValidityChecks(chain_info_.joint_names,
                                              collision_operations_,
                                              allowed_contacts_,
                                              emp,
                                              constraints_,
                                              link_padding_,
                                              response.error_code);
  //can't transform constraints
  if(response.error_code.val == response.error_code.FRAME_TRANSFORM_FAILURE) {
    return true;
  }
  
  kinematic_state_ = new planning_models::KinematicState(planning_monitor_->getKinematicModel());
  planning_monitor_->setRobotStateAndComputeTransforms(ik_request_.robot_state, *kinematic_state_);
  ik_request_.pose_stamped = pose_msg_out;
  ROS_DEBUG("Transformed IK request is in the frame: %s",ik_request_.pose_stamped.header.frame_id.c_str());
  arm_kinematics_constraint_aware::reorderJointState(ik_request_.ik_seed_state.joint_state,chain_info_);

  ros::Time ik_solver_time = ros::Time::now();
  int kinematics_error_code;
  bool ik_valid = (kinematics_solver_->searchPositionIK(ik_request_.pose_stamped.pose,
                                                        ik_request_.ik_seed_state.joint_state.position,
                                                        request_in.timeout.toSec(),
                                                        response.solution.joint_state.position,
                                                        boost::bind(&ArmKinematicsConstraintAware::initialPoseCheck, this, _1, _2, _3),
                                                        boost::bind(&ArmKinematicsConstraintAware::collisionCheck, this, _1, _2, _3),kinematics_error_code));
  ROS_INFO("IK solver time: %f",(ros::Time::now()-ik_solver_time).toSec());

  planning_monitor_->revertToDefaultState();
  if(ik_valid)
  {
    response.solution.joint_state.name = chain_info_.joint_names;
    if(visualize_solution_)
    {
      motion_planning_msgs::DisplayTrajectory display_trajectory;
      display_trajectory.trajectory.joint_trajectory.points.resize(1);
      display_trajectory.trajectory.joint_trajectory.points[0].positions = response.solution.joint_state.position;
      display_trajectory.trajectory.joint_trajectory.joint_names = response.solution.joint_state.name;
      planning_monitor_->convertKinematicStateToRobotState(*kinematic_state_,display_trajectory.robot_state);
      display_trajectory_publisher_.publish(display_trajectory);
    }
    ROS_INFO("IK service time: %f",(ros::Time::now()-start_time).toSec());
    response.error_code.val = response.error_code.SUCCESS;
    delete kinematic_state_;
    return true;
  }
  else
  {
    ROS_ERROR("An IK solution could not be found");
    response.error_code = kinematicsErrorCodeToMotionPlanningErrorCode(kinematics_error_code);
    if(response.error_code.val != response.error_code.IK_LINK_IN_COLLISION) 
    {
      sendEndEffectorPose(kinematic_state_,true);
    }
    delete kinematic_state_;
    return true;
  }
}

void ArmKinematicsConstraintAware::collisionCheck(const geometry_msgs::Pose &ik_pose,
                                                  const std::vector<double> &ik_solution,
                                                  int &error_code)
{
  std::map<std::string, double> joint_values;
  for(unsigned int i=0; i < chain_info_.joint_names.size(); i++)
    joint_values[chain_info_.joint_names[i]] = ik_solution[i];

  kinematic_state_->setKinematicState(joint_values);
  planning_monitor_->getEnvironmentModel()->updateRobotModel(kinematic_state_);

  bool check = (!planning_monitor_->getEnvironmentModel()->isCollision() &&  !planning_monitor_->getEnvironmentModel()->isSelfCollision());
  if(!check) 
  {
    planning_monitor_->broadcastCollisions();
    error_code = kinematics::STATE_IN_COLLISION;
  }
  else {
    error_code = kinematics::SUCCESS;
  }
  
  if(!planning_monitor_->checkGoalConstraints(kinematic_state_, false)) {
    error_code = kinematics::GOAL_CONSTRAINTS_VIOLATED;
    ROS_INFO("Constraints violated at current state");
  }
}

void ArmKinematicsConstraintAware::initialPoseCheck(const geometry_msgs::Pose &ik_pose,
                                                    const std::vector<double> &ik_solution,
                                                    int &error_code)
{
  std::string kinematic_frame_id = kinematics_solver_->getBaseFrame();
  std::string planning_frame_id = planning_monitor_->getWorldFrameId();
  geometry_msgs::PoseStamped pose_stamped;
  pose_stamped.pose = ik_pose;
  pose_stamped.header.stamp = ros::Time::now();
  pose_stamped.header.frame_id = kinematic_frame_id;
  if (!tf_.canTransform(planning_frame_id,
                        pose_stamped.header.frame_id,
                        pose_stamped.header.stamp))
  {
    std::string err;
    ros::Time tmp;
    if(tf_.getLatestCommonTime(pose_stamped.header.frame_id,planning_frame_id,tmp,&err) != tf::NO_ERROR)
    {
      ROS_ERROR("Cannot transform from '%s' to '%s'. TF said: %s",pose_stamped.header.frame_id.c_str(),planning_frame_id.c_str(), err.c_str());
    }
    else
      pose_stamped.header.stamp = tmp;
  }
    
  try
  {
    tf_.transformPose(planning_frame_id,pose_stamped,pose_stamped);
  }
  catch(tf::TransformException& ex)
  {
    ROS_ERROR("Cannot transform from '%s' to '%s'. Tf said: %s",
              pose_stamped.header.frame_id.c_str(),planning_frame_id.c_str(), ex.what());
    error_code = kinematics::FRAME_TRANSFORM_FAILURE;
    return;
  }
  
  //need to disable collisions correctly, and re-enable at the end
  std::vector<std::vector<bool> > orig_allowed;
  std::map<std::string, unsigned int> vecIndices;
  planning_monitor_->getEnvironmentModel()->getCurrentAllowedCollisionMatrix(orig_allowed, vecIndices);

  std::vector<std::vector<bool> > new_allowed = orig_allowed;
  for(unsigned int i = 0; i < arm_links_.size(); i++) {
    unsigned int vind = vecIndices[arm_links_[i]];
    for(unsigned int j = 0; j < new_allowed.size(); j++) {
      new_allowed[vind][j] = true;
      new_allowed[j][vind] = true;
    }
  }
  //planning_monitor_->printAllowedCollisionMatrix(orig_allowed, vecIndices);
  //planning_monitor_->printAllowedCollisionMatrix(new_allowed, vecIndices);
  planning_monitor_->getEnvironmentModel()->setAllowedCollisionMatrix(new_allowed, vecIndices);

  btTransform transform;
  tf::poseMsgToTF(pose_stamped.pose,transform);
  if(!kinematic_state_->hasLinkState(ik_request_.ik_link_name)) {
    ROS_ERROR("Could not find end effector root_link %s", ik_request_.ik_link_name.c_str());
    error_code  = kinematics::INVALID_LINK_NAME;
    return;
  }
  kinematic_state_->updateKinematicStateWithLinkAt(ik_request_.ik_link_name, transform);
  planning_monitor_->getEnvironmentModel()->updateRobotModel(kinematic_state_);

  bool check = ( !planning_monitor_->getEnvironmentModel()->isCollision() &&  
            !planning_monitor_->getEnvironmentModel()->isSelfCollision() );
  if(!check) {
    error_code  = kinematics::IK_LINK_IN_COLLISION;
    planning_monitor_->broadcastCollisions();
    sendEndEffectorPose(kinematic_state_, false);
  }
  else
    error_code = kinematics::SUCCESS;
    
  planning_monitor_->getEnvironmentModel()->setAllowedCollisionMatrix(orig_allowed, vecIndices);
  ROS_DEBUG("Initial pose check done with result %s",check ? "not_in_collision" : "in_collision" );
}

void ArmKinematicsConstraintAware::sendEndEffectorPose(const planning_models::KinematicState* state, bool valid) {
  if(!robot_model_initialized_) return;
  visualization_msgs::MarkerArray hand_array;
  unsigned int id = 0;
  for(unsigned int i = 0; i < end_effector_collision_links_.size(); i++) {
    boost::shared_ptr<const urdf::Link> urdf_link = robot_model_.getLink(end_effector_collision_links_[i]);
    if(urdf_link == NULL) {
      ROS_DEBUG_STREAM("No entry in urdf for link " << end_effector_collision_links_[i]);
      continue;
    }
    if(!urdf_link->collision) {
      continue;
    }
    const urdf::Geometry *geom = urdf_link->collision->geometry.get();
    if(!geom) {
      ROS_DEBUG_STREAM("No collision geometry for link " << end_effector_collision_links_[i]);
      continue;
    }
    const urdf::Mesh *mesh = dynamic_cast<const urdf::Mesh*>(geom);
    if(mesh) {
      if (!mesh->filename.empty()) {
        planning_models::KinematicState::LinkState* ls = state->getLinkState(end_effector_collision_links_[i]);
        visualization_msgs::Marker mark;
        mark.header.frame_id = planning_monitor_->getWorldFrameId();
        mark.header.stamp = ros::Time::now();
        mark.id = id++;
        if(!valid) {
          mark.ns = "initial_pose_collision";
        } else {
          mark.ns = "initial_pose_ok";
        }
        mark.type = mark.MESH_RESOURCE;
        mark.scale.x = 1.0;
        mark.scale.y = 1.0;
        mark.scale.z = 1.0;
        if(!valid) {
          mark.color.r = 1.0;
        } else {
          mark.color.g = 1.0;
        }
        mark.color.a = .8;
        mark.pose.position.x = ls->getGlobalCollisionBodyTransform().getOrigin().x();
        mark.pose.position.y = ls->getGlobalCollisionBodyTransform().getOrigin().y();
        mark.pose.position.z = ls->getGlobalCollisionBodyTransform().getOrigin().z();
        mark.pose.orientation.x = ls->getGlobalCollisionBodyTransform().getRotation().x();
        mark.pose.orientation.y = ls->getGlobalCollisionBodyTransform().getRotation().y();
        mark.pose.orientation.z = ls->getGlobalCollisionBodyTransform().getRotation().z();
        mark.pose.orientation.w = ls->getGlobalCollisionBodyTransform().getRotation().w();
        mark.mesh_resource = mesh->filename;
        hand_array.markers.push_back(mark);
      }
    }
  }
  vis_marker_array_publisher_.publish(hand_array);
}

void ArmKinematicsConstraintAware::printStringVec(const std::string &prefix, const std::vector<std::string> &string_vector)
{
  ROS_DEBUG("%s",prefix.c_str());
  for(unsigned int i=0; i < string_vector.size(); i++)
  {
    ROS_DEBUG("%s",string_vector[i].c_str());
  }
}

bool ArmKinematicsConstraintAware::setupCollisionEnvironment()
{
  node_handle_.param<bool>("use_collision_map", use_collision_map_, true);

  std::string urdf_xml,full_urdf_xml;
  root_handle_.param("urdf_xml", urdf_xml, std::string("robot_description"));
  if(!root_handle_.getParam(urdf_xml,full_urdf_xml))
  {
    ROS_ERROR("Could not load the xml from parameter server: %s\n", urdf_xml.c_str());
    robot_model_initialized_ = false;
  }
  else
  {
    robot_model_.initString(full_urdf_xml);
    robot_model_initialized_ = true;
  }

  // monitor robot
  collision_models_ = new planning_environment::CollisionModels("robot_description");

  if(!collision_models_)
  {
    ROS_INFO("Could not initialize collision models");
    return false;
  }
  planning_monitor_ = new planning_environment::PlanningMonitor(collision_models_, &tf_);

  if(!planning_monitor_)
  {
    ROS_ERROR("Could not initialize planning monitor");
    return false;
  }
  else
  {
    ROS_INFO("Initialized planning monitor");
  }

  planning_monitor_->setUseCollisionMap(use_collision_map_);
  planning_monitor_->startEnvironmentMonitor();
  if(!collision_models_->loadedModels())
  {
    ROS_ERROR("Could not load models");
    return false;
  }
  if (!collision_models_->getKinematicModel()->hasModelGroup(group_))
  {
    ROS_ERROR("Group '%s' is not known", group_.c_str());
    return false;
  }
  else
    ROS_INFO("Configuring action for '%s'", group_.c_str());

  const std::vector<const planning_models::KinematicModel::JointModel*>& p_joints = collision_models_->getKinematicModel()->getModelGroup(group_)->getJointModels();
  for(unsigned int i=0; i < p_joints.size(); i++)
  {
    default_collision_links_.push_back(p_joints[i]->getChildLinkModel()->getName()); 
  }

  //getting arm link names
  arm_links_ = collision_models_->getPlanningGroupLinks().at(group_);

  if (planning_monitor_->getExpectedJointStateUpdateInterval() > 1e-3)
    planning_monitor_->waitForState();
  if (planning_monitor_->getExpectedMapUpdateInterval() > 1e-3 && use_collision_map_)
    planning_monitor_->waitForMap();
  //planning_monitor_->setCollisionCheckOnlyLinks(default_collision_links_,true);

  end_effector_collision_links_.clear();
  const planning_models::KinematicModel::LinkModel* end_effector_link = planning_monitor_->getKinematicModel()->getLinkModel(chain_info_.link_names.back());
  std::vector<const planning_models::KinematicModel::LinkModel*> tempLinks;
  planning_monitor_->getKinematicModel()->getChildLinkModels(end_effector_link, tempLinks);
  for (unsigned int i = 0 ; i < tempLinks.size() ; ++i)
    end_effector_collision_links_.push_back(tempLinks[i]->getName());
  for(unsigned int i=0; i < end_effector_collision_links_.size(); i++)
    default_collision_links_.push_back(end_effector_collision_links_[i]);

  printStringVec("Default collision links",default_collision_links_);
  printStringVec("End effector links",end_effector_collision_links_);

  ROS_DEBUG("Root link name is: %s",root_name_.c_str());

  planning_monitor_->setOnCollisionContactCallback(boost::bind(&ArmKinematicsConstraintAware::contactFound, this, _1));
  vis_marker_publisher_ = root_handle_.advertise<visualization_msgs::Marker>("kinematics_collisions", 128);
  vis_marker_array_publisher_ = root_handle_.advertise<visualization_msgs::MarkerArray>("kinematics_collisions_array", 128);

  setup_collision_environment_ = true;
  return true;
}

/** \brief The ccost and display arguments should be bound by the caller. This is a callback function that gets called by the planning
 * environment when a collision is found */
void ArmKinematicsConstraintAware::contactFound(collision_space::EnvironmentModel::Contact &contact)
{

  static int count = 0;
  
  std::string ns_name;
  if(contact.link1 != NULL) {
    //ROS_INFO_STREAM("Link 1 is " << contact.link2->name);
    if(contact.link1_attached_body_index == 0) {
      ns_name += contact.link1->getName()+"+";
    } else {
      if(contact.link1->getAttachedBodyModels().size() < contact.link1_attached_body_index) {
        ROS_ERROR("Link doesn't have attached body with indicated index");
      } else {
        ns_name += contact.link1->getAttachedBodyModels()[contact.link1_attached_body_index-1]->getName()+"+";
      }
    }
  } 
  
  if(contact.link2 != NULL) {
    //ROS_INFO_STREAM("Link 2 is " << contact.link2->name);
    if(contact.link2_attached_body_index == 0) {
      ns_name += contact.link2->getName();
    } else {
      if(contact.link2->getAttachedBodyModels().size() < contact.link2_attached_body_index) {
        ROS_ERROR("Link doesn't have attached body with indicated index");
      } else {
        ns_name += contact.link2->getAttachedBodyModels()[contact.link2_attached_body_index-1]->getName();
      }
    }
  } 
  
  if(!contact.object_name.empty()) {
    //ROS_INFO_STREAM("Object is " << contact.object_name);
    ns_name += contact.object_name;
  }
  
  visualization_msgs::Marker mk;
  mk.header.stamp = planning_monitor_->lastPoseUpdate();
  mk.header.frame_id = planning_monitor_->getWorldFrameId();
  mk.ns = ns_name;
  mk.id = count++;
  mk.type = visualization_msgs::Marker::SPHERE;
  mk.action = visualization_msgs::Marker::ADD;
  mk.pose.position.x = contact.pos.x();
  mk.pose.position.y = contact.pos.y();
  mk.pose.position.z = contact.pos.z();
  mk.pose.orientation.w = 1.0;
  
  mk.scale.x = mk.scale.y = mk.scale.z = 0.01;
  
  mk.color.a = 0.6;
  mk.color.r = 1.0;
  mk.color.g = 0.04;
  mk.color.b = 0.04;
  
  //mk.lifetime = ros::Duration(30.0);
  
  vis_marker_publisher_.publish(mk);
}

bool ArmKinematicsConstraintAware::getPositionIK(kinematics_msgs::GetPositionIK::Request &request, 
                                                 kinematics_msgs::GetPositionIK::Response &response)
{
  if(!active_)
  {
    ROS_ERROR("IK service not active");
    return true;
  }

  if(!checkIKService(request,response,chain_info_))
    return true;

  geometry_msgs::PoseStamped pose_msg_in = request.ik_request.pose_stamped;
  geometry_msgs::PoseStamped pose_msg_out;
  if(!convertPoseToRootFrame(pose_msg_in,pose_msg_out,root_name_,tf_))
  {
    response.error_code.val = response.error_code.FRAME_TRANSFORM_FAILURE;
    return true;
  }
  arm_kinematics_constraint_aware::reorderJointState(request.ik_request.ik_seed_state.joint_state,chain_info_);

  int kinematics_error_code;
  bool ik_valid = kinematics_solver_->searchPositionIK(pose_msg_out.pose,
                                                      request.ik_request.ik_seed_state.joint_state.position,
                                                      request.timeout.toSec(),
                                                      response.solution.joint_state.position,
                                                      kinematics_error_code);

  response.error_code = kinematicsErrorCodeToMotionPlanningErrorCode(kinematics_error_code);

  if(ik_valid)
  {
    response.solution.joint_state.name = chain_info_.joint_names;
    response.error_code.val = response.error_code.SUCCESS;
    return true;
  }
  else
  {
    ROS_DEBUG("An IK solution could not be found");   
    return true;
  }
}

bool ArmKinematicsConstraintAware::getIKSolverInfo(kinematics_msgs::GetKinematicSolverInfo::Request &request, 
                                                     kinematics_msgs::GetKinematicSolverInfo::Response &response)
{
  if(!active_)
  {
    ROS_ERROR("IK node not active");
    return true;
  }
  response.kinematic_solver_info = chain_info_;
  return true;
}

bool ArmKinematicsConstraintAware::getFKSolverInfo(kinematics_msgs::GetKinematicSolverInfo::Request &request, 
                                                     kinematics_msgs::GetKinematicSolverInfo::Response &response)
{
  if(!active_)
  {
    ROS_ERROR("IK node not active");
    return true;
  }
  response.kinematic_solver_info = chain_info_;
  return true;
}

bool ArmKinematicsConstraintAware::getPositionFK(kinematics_msgs::GetPositionFK::Request &request, 
                                                   kinematics_msgs::GetPositionFK::Response &response)
{
  if(!active_)
  {
    ROS_ERROR("FK service not active");
    return true;
  }

  if(!checkFKService(request,response,chain_info_))
    return true;

  arm_kinematics_constraint_aware::reorderJointState(request.robot_state.joint_state,chain_info_);

  response.pose_stamped.resize(request.fk_link_names.size());
  response.fk_link_names.resize(request.fk_link_names.size());

  bool valid = true;
  std::vector<geometry_msgs::Pose> solutions;
  solutions.resize(request.fk_link_names.size());
  if(kinematics_solver_->getPositionFK(request.fk_link_names,request.robot_state.joint_state.position,solutions) >=0)
  {    
    for(unsigned int i=0; i < solutions.size(); i++)
    {      
      geometry_msgs::PoseStamped pose_stamped;
      pose_stamped.pose = solutions[i];
      pose_stamped.header.frame_id = root_name_;
      pose_stamped.header.stamp = ros::Time();
      try
      {
        tf_.transformPose(request.header.frame_id,pose_stamped,pose_stamped);
      }
      catch(...)
      {
        ROS_ERROR("Could not transform FK pose to frame: %s",request.header.frame_id.c_str());
        response.error_code.val = response.error_code.FRAME_TRANSFORM_FAILURE;
        return false;
      }
      response.pose_stamped[i] = pose_stamped;
      response.fk_link_names[i] = request.fk_link_names[i];
      response.error_code.val = response.error_code.SUCCESS;
    }
  }
  else
  {
    ROS_ERROR("Could not compute FK");
    response.error_code.val = response.error_code.NO_FK_SOLUTION;
    valid = false;
  }
  return valid;
}

} // namespace

