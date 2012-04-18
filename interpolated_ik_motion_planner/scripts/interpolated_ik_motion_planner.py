#!/usr/bin/python
# Software License Agreement (BSD License)
#
# Copyright (c) 2009, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of the Willow Garage nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# author: Kaijen Hsiao

## @package interpolated_ik_motion_planner
# Server to provide interpolated IK motion plans
# relevant params:
# num_steps: the number of steps to use when interpolating including start and finish (0 means use pos_spacing and 
#     rot_spacing to calculate the number of steps, anything ==1 or <0 will become 2, start and finish)
# consistent_angle: the max angle distance (in any joint) before we declare that the path is inconsistent
# collision_check_resolution: how many steps between collision checks (0 or 1 is check every step; 2 is every other, etc.)
# steps_before_abort: the number of steps in the plan (starting from the end and going backwards) that can be invalid 
#     due to collisions or inconsistency before aborting (0 is abort as soon as you find one, 1 is allow one and still 
#     continue, -1 or >=num_steps to never abort early)
# pos_spacing: the max translation (m) to move the wrist between waypoints (only used if num_steps is 0)
# rot_spacing: the max rotation (rad) to move the wrist between waypoints (only used if num_steps is 0)
# collision_aware: if this is 0, collisions won't be checked for (returns non-collision aware IK solutions)
# start_from_end: if this is 1, the planner searches for an IK solution for the end first, then works backwards from there
# max_joint_vels: a list of maximum joint velocities to use when computing times and velocities for the joint trajectory (defaults to [.2]*7 if left empty)
# max_joint_accs: a list of maximum accelerations to use when computing times and velocities for the joint trajectory (defaults to [.5]*7 if left empty)

# Parameters can be changed by calling the r_interpolated_ik_motion_plan_set_params service (or l_inter...)


# The main service message is GetMotionPlan.srv, in arm_navigation_msgs.  Parts that were hijacked 
# for the relevant inputs and outputs:

# inputs:
# req.motion_plan_request.start_state.joint_state.name: the names of the arm joints in order (defaults to those provided by IKQuery, if not specified)
# req.motion_plan_request.start_state.joint_state.position: a set of joint angles to stay sort of close to if possible (defaults to current joint angles if not specified)

# req.motion_plan_request.start_state.multi_dof_joint_state.pose: the start pose for the r_wrist_roll_link
# req.motion_plan_request.start_state.multi_dof_joint_state.child_frame_id: the link to put at that pose (better be 'r_wrist_roll_link' or 'l_wrist_roll_link')
# req.motion_plan_request.start_state.multi_dof_joint_state.frame_id: the frame for the start pose

# req.motion_plan_request.goal_constraints.position_constraints[0].position: the (x,y,z) goal position of the same link as the start pose
# req.motion_plan_request.goal_constraints.position_constraints[0].header.frame_id: the frame that goal position is in
# req.motion_plan_request.goal_constraints.orientation_constraints[0].orientation: the (x,y,z,w) goal orientation of the same link as the start pose
# req.motion_plan_request.goal_constraints.orientation_constraints[0].header.frame_id: the frame that goal orientation is in

# req.motion_plan_request.workspace_parameters.ordered_collision_operations: the list of collision operations to modify the collision space
# req.motion_plan_request.link_padding: links and collision-space padding to change from the default dynamically

# outputs:
# res.trajectory.joint_trajectory.joint_names: the arm joint names in the normal order, as spit out by IKQuery
# res.trajectory.joint_trajectory.points: a list of JointTrajectoryPoints whose 'positions' fields are lists of joint angles in a trajectory that gets you from start to goal (positions will be all 0s if a point on the path is out of reach),
# 'velocities' fields are reasonable joint velocities, and 'time_from_start' fields are reasonable times to get to each waypoint. ('accelerations' will be empty.)
# res.trajectory_error_codes: a list of ArmNavigationErrorCodes messages the same length as the trajectory, 
# with values for each trajectory point as follows:
# ArmNavigationErrorCodes.SUCCESS (1): no problem
# ArmNavigationErrorCodes.COLLISION_CONSTRAINTS_VIOLATED (-23): no non-colliding IK solution (colliding solution provided)
# ArmNavigationErrorCodes.PATH_CONSTRAINTS_VIOLATED (-20): inconsistency in path between this point and the next point
# ArmNavigationErrorCodes.JOINT_LIMITS_VIOLATED (-21): out of reach (no colliding solution)
# ArmNavigationErrorCodes.PLANNING_FAILED (-1): aborted before getting to this point

# Everything else is completely ignored.


import roslib; roslib.load_manifest('interpolated_ik_motion_planner')
import rospy
import interpolated_ik_motion_planner.ik_utilities as ik_utilities
import math
import sys
import pdb
from arm_navigation_msgs.srv import GetMotionPlan, GetMotionPlanResponse
from arm_navigation_msgs.msg import ArmNavigationErrorCodes, RobotState
from geometry_msgs.msg import PoseStamped, PointStamped, QuaternionStamped, Pose, Point, Quaternion
from trajectory_msgs.msg import JointTrajectoryPoint
from interpolated_ik_motion_planner.srv import SetInterpolatedIKMotionPlanParams, SetInterpolatedIKMotionPlanParamsResponse
from sensor_msgs.msg import JointState

# class to provide the interpolated ik motion planner service
class InterpolatedIKService:

    def __init__(self, which_arm): #which_arm is 'r' or 'l'
        self.which_arm = which_arm 
        self.node_name = which_arm+'_interpolated_ik_motion_planner_server'

        rospy.init_node(self.node_name)

        #get parameters from the parameter server

        #the number of steps to use when interpolating (including start and finish)
        self.num_steps = rospy.get_param(self.node_name+'/num_steps', 6)

        #the max angle distance (in any joint) before we declare that the path is inconsistent
        self.consistent_angle = rospy.get_param(self.node_name+'/consistent_angle', math.pi/9) 

        #how many steps between collision checks (0 or 1 is check every step; 2 is every other, etc.)
        self.collision_check_resolution = rospy.get_param(self.node_name+'/collision_check_resolution', 1)

        #how many steps in the plan can be invalid before aborting
        self.steps_before_abort = rospy.get_param(self.node_name+'/steps_before_abort', 0)

        #the max translation (m) to move the wrist between waypoints (only used if num_steps is 0)
        self.pos_spacing = rospy.get_param(self.node_name+'/pos_spacing', 0.01)

        #the max rotation (rad) to move the wrist between waypoints (only used if num_steps is 0)
        self.rot_spacing = rospy.get_param(self.node_name+'/rot_spacing', .1)

        #whether to check for collisions or not
        self.collision_aware = rospy.get_param(self.node_name+'/collision_aware', 1)

        #if 1, starts the IK calculation from the end and works backwards (if 0, starts from the start)
        self.start_from_end = rospy.get_param(self.node_name+'/start_from_end', 0)

        #max joint velocities to use when calculating times and vels for the trajectory
        self.max_joint_vels = rospy.get_param(self.node_name+'/max_joint_vels', [.1]*7)

        #max joint accelerations to use when calculating times and vels for the trajectory
        self.max_joint_accs = rospy.get_param(self.node_name+'/max_joint_vels', [.25]*7)

        #initialize an IKUtilities class object
        if which_arm == 'r':
            self.ik_utils = ik_utilities.IKUtilities('right')
        else:
            self.ik_utils = ik_utilities.IKUtilities('left')            

        #advertise interpolated IK service
        s1 = rospy.Service(which_arm+'_interpolated_ik_motion_plan', \
                GetMotionPlan, self.interpolated_ik_motion_planner_callback)

        #advertise param changing service
        s2 = rospy.Service(which_arm+'_interpolated_ik_motion_plan_set_params', \
                SetInterpolatedIKMotionPlanParams, self.set_params_callback)


    ##add a header to a message with a 0 timestamp (good for getting the latest TF transform)
    def add_header(self, msg, frame):
        msg.header.frame_id = frame
        msg.header.stamp = rospy.Time()
        return msg


    ##pretty-print list to string
    def pplist(self, list):
        return ' '.join(['%2.3f'%x for x in list])


    ##callback for the set_params service
    def set_params_callback(self, req):
        self.num_steps = req.num_steps
        self.consistent_angle = req.consistent_angle
        self.collision_check_resolution= req.collision_check_resolution
        self.steps_before_abort = req.steps_before_abort
        self.pos_spacing = req.pos_spacing
        self.rot_spacing = req.rot_spacing
        self.collision_aware = req.collision_aware
        self.start_from_end = req.start_from_end
        self.max_joint_vels = req.max_joint_vels
        self.max_joint_accs = req.max_joint_accs
        return SetInterpolatedIKMotionPlanParamsResponse()


    ##callback for get_interpolated_ik_motion_plan service
    def interpolated_ik_motion_planner_callback(self, req):

        #names and angles for the joints in their desired order
        joint_names = req.motion_plan_request.start_state.joint_state.name
        start_angles = req.motion_plan_request.start_state.joint_state.position

        #sanity-checking: joint_names and start_angles should be the same length, if any start_angles are specified
        if start_angles and len(joint_names) != len(start_angles):
            rospy.logerr("start_state.joint_state.name needs to be the same length as start_state.joint_state.position!  Quitting")
            return 0
    
        #reorder the start angles to the order needed by IK
        reordered_start_angles = []

        #get the current joint states for the robot
        #joint_states_msg = rospy.wait_for_message('joint_states', JointState, 10.0)
        #if not joint_states_msg:
        #    rospy.logerr("unable to get joint_states message")
        #    return 0

        #get the desired start angles for each IK arm joint in turn from start_state.joint_state.position
        #(use the current angle if not specified)
        for joint_name in self.ik_utils.joint_names:

            #desired start angle specified
            if joint_name in joint_names and start_angles:
                index = joint_names.index(joint_name)
                reordered_start_angles.append(start_angles[index])
            else:
                rospy.logerr("missing joint angle, can't deal")
                return 0

            #desired start angle not specified, use the current angle
#elif 0: #joint_name in joint_states_msg.name:
#                index = joint_states_msg.name.index(joint_name)
#                current_position = joint_states_msg.position[index]
#                reordered_start_angles.append(current_position)

            #malformed joint_states message?
#            else:
#                rospy.logerr("an expected arm joint,"+joint_name+"was not found!")
#                return 0

        #get additional desired joint angles (such as for the gripper) to pass through to IK
        additional_joint_angles = []
        additional_joint_names = []
        for (ind, joint_name) in enumerate(joint_names):
            if joint_name not in self.ik_utils.joint_names:
                #rospy.loginfo("found %s"%joint_name)
                additional_joint_angles.append(start_angles[ind])
                additional_joint_names.append(joint_name)
        IK_robot_state = None
        if additional_joint_angles:
            #rospy.loginfo("adding additional start angles for:"+str(additional_joint_names))
            #rospy.loginfo("additional joint angles:"+str(additional_joint_angles))
            IK_robot_state = RobotState()
            IK_robot_state.joint_state.name = additional_joint_names
            IK_robot_state.joint_state.position = additional_joint_angles

        #check that the desired link is in the list of possible IK links (only r/l_wrist_roll_link for now)
        link_name = req.motion_plan_request.start_state.multi_dof_joint_state.child_frame_ids[0]
        if link_name != self.ik_utils.link_name:
            rospy.logerr("link_name not allowed: %s"%link_name)
            return 0

        #the start pose for that link
        start_pose = req.motion_plan_request.start_state.multi_dof_joint_state.poses[0]

        #the frame that start pose is in
        frame_id = req.motion_plan_request.start_state.multi_dof_joint_state.frame_ids[0]

        #turn it into a PoseStamped
        start_pose_stamped = self.add_header(PoseStamped(), frame_id)
        start_pose_stamped.pose = start_pose

        #the desired goal position
        goal_pos = req.motion_plan_request.goal_constraints.position_constraints[0].position         
        
        #the frame that goal position is in
        goal_pos_frame = req.motion_plan_request.goal_constraints.position_constraints[0].header.frame_id

        #convert the position to base_link frame
        goal_ps = self.add_header(PointStamped(), goal_pos_frame)
        goal_ps.point = goal_pos
        goal_pos_list = self.ik_utils.point_stamped_to_list(goal_ps, 'base_link')

        #the desired goal orientation
        goal_quat = req.motion_plan_request.goal_constraints.orientation_constraints[0].orientation

        #the frame that goal orientation is in
        goal_quat_frame = req.motion_plan_request.goal_constraints.orientation_constraints[0].header.frame_id 

        #convert the quaternion to base_link frame
        goal_qs = self.add_header(QuaternionStamped(), goal_quat_frame)
        goal_qs.quaternion = goal_quat
        goal_quat_list = self.ik_utils.quaternion_stamped_to_list(goal_qs, 'base_link')

        #assemble the goal pose into a PoseStamped
        goal_pose_stamped = self.add_header(PoseStamped(), 'base_link')
        goal_pose_stamped.pose = Pose(Point(*goal_pos_list), Quaternion(*goal_quat_list))

        #get the ordered collision operations, if there are any
        ordered_collision_operations = None #req.motion_plan_request.ordered_collision_operations
        #if ordered_collision_operations.collision_operations == []:
        #    ordered_collision_operations = None

        #get the link paddings, if there are any
        link_padding = None #req.motion_plan_request.link_padding
        #if link_padding == []:
        #    link_padding = None

        #RUN!  Check the Cartesian path for consistent, non-colliding IK solutions
        (trajectory, error_codes) = self.ik_utils.check_cartesian_path(start_pose_stamped, \
                 goal_pose_stamped, reordered_start_angles, self.pos_spacing, self.rot_spacing, \
                 self.consistent_angle, self.collision_aware, self.collision_check_resolution, \
                 self.steps_before_abort, self.num_steps, ordered_collision_operations, \
                 self.start_from_end, IK_robot_state, link_padding)

        #find appropriate velocities and times for the valid part of the resulting joint path (invalid parts set to 0)
        #if we're searching from the end, keep the end; if we're searching from the start, keep the start
        start_ind = 0
        stop_ind = len(error_codes)
        if self.start_from_end:
            for ind in range(len(error_codes)-1, 0, -1):
                if error_codes[ind]:
                    start_ind = ind+1
                    break
        else:
            for ind in range(len(error_codes)):
                if error_codes[ind]:
                    stop_ind = ind
                    break
        (times, vels) = self.ik_utils.trajectory_times_and_vels(trajectory[start_ind:stop_ind], self.max_joint_vels, self.max_joint_accs)
        times = [0]*start_ind + times + [0]*(len(error_codes)-stop_ind)
        vels = [[0]*7]*start_ind + vels + [[0]*7]*(len(error_codes)-stop_ind)

        rospy.logdebug("trajectory:")
        for ind in range(len(trajectory)):
            rospy.logdebug("error code "+ str(error_codes[ind]) + " pos : " + self.pplist(trajectory[ind]))
        rospy.logdebug("")
        for ind in range(len(trajectory)):
            rospy.logdebug("time: " + "%5.3f  "%times[ind] + "vels: " + self.pplist(vels[ind]))

        #the response
        res = GetMotionPlanResponse()

        #the arm joint names in the normal order, as spit out by IKQuery
        res.trajectory.joint_trajectory.joint_names = self.ik_utils.joint_names[:]

        #a list of 7-lists of joint angles, velocities, and times for a trajectory that gets you from start to goal 
        #(all 0s if there was no IK solution for a point on the path)
        res.trajectory.joint_trajectory.points = []
        for i in range(len(trajectory)):
            joint_trajectory_point = JointTrajectoryPoint()
            joint_trajectory_point.positions = trajectory[i]
            joint_trajectory_point.velocities = vels[i]
            joint_trajectory_point.time_from_start = rospy.Duration(times[i])
            res.trajectory.joint_trajectory.points.append(joint_trajectory_point)

        #a list of ArmNavigationErrorCodes messages, one for each trajectory point, with values as follows:
        #ArmNavigationErrorCodes.SUCCESS (1): no problem
        #ArmNavigationErrorCodes.COLLISION_CONSTRAINTS_VIOLATED (-23): no non-colliding IK solution (colliding solution provided)
        #ArmNavigationErrorCodes.PATH_CONSTRAINTS_VIOLATED (-20): inconsistency in path between this point and the next point
        #ArmNavigationErrorCodes.JOINT_LIMITS_VIOLATED (-21): out of reach (no colliding solution)
        #ArmNavigationErrorCodes.PLANNING_FAILED (0): aborted before getting to this point
        error_code_dict = {0:ArmNavigationErrorCodes.SUCCESS, 1:ArmNavigationErrorCodes.COLLISION_CONSTRAINTS_VIOLATED, \
                           2:ArmNavigationErrorCodes.PATH_CONSTRAINTS_VIOLATED, 3:ArmNavigationErrorCodes.JOINT_LIMITS_VIOLATED, \
                           4:ArmNavigationErrorCodes.PLANNING_FAILED}

        trajectory_error_codes = [ArmNavigationErrorCodes(val=error_code_dict[error_code]) for error_code in error_codes]
        res.trajectory_error_codes = trajectory_error_codes 
        res.error_code.val = ArmNavigationErrorCodes.SUCCESS
#         rospy.loginfo("trajectory:")
#         for ind in range(len(trajectory)):
#             rospy.loginfo("error code "+ str(error_codes[ind]) + " pos : " + self.pplist(trajectory[ind]))
#         rospy.loginfo("")
#         for ind in range(len(trajectory)):
#             rospy.loginfo("time: " + "%5.3f  "%times[ind] + "vels: " + self.pplist(vels[ind]))

        return res


if __name__ == "__main__":

    if len(sys.argv) < 2 or sys.argv[1] != 'r' and sys.argv[1] != 'l':
        rospy.logerr("usage: interpolated_ik_motion_planner.py which_arm (which_arm is r or l)")
        sys.exit(1)

    which_arm = sys.argv[1]
    interpolated_ik_service = InterpolatedIKService(which_arm)
    rospy.loginfo("Ready to serve interpolated IK motion plan requests.")

    rospy.spin()    

