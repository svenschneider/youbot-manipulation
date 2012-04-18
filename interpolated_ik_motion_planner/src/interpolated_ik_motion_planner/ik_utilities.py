#! /usr/bin/python
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

## @package ik_utilities
#Utility functions for doing inverse kinematics, forward kinematics, checking a Cartesian path

import roslib; roslib.load_manifest('interpolated_ik_motion_planner')
import rospy
from kinematics_msgs.srv import GetKinematicSolverInfo, GetPositionIK, GetPositionFK, GetConstraintAwarePositionIK, GetConstraintAwarePositionIKRequest
from kinematics_msgs.msg import PositionIKRequest
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion, PointStamped, Vector3Stamped
from visualization_msgs.msg import Marker
from arm_navigation_msgs.msg import RobotState, MultiDOFJointState, ArmNavigationErrorCodes
from arm_navigation_msgs.srv import GetStateValidity, GetStateValidityRequest
from sensor_msgs.msg import JointState
import math
import random
import time
import tf
import numpy
import pdb

#pretty-print list to string
def pplist(list):
    return ' '.join(['%8.5f'%x for x in list])

#utility functions for doing inverse kinematics, forward kinematics, checking a Cartesian path
class IKUtilities:

    #initialize all service functions
    #if wait_for_services = 0, you must call check_services_and_get_ik_info externally before running any of the IK/FK functions
    def __init__(self, whicharm, tf_listener = None, wait_for_services = 1): #whicharm is 'right' or 'left'
        
        #gets the robot_prefix from the parameter server. Default is pr2 
        robot_prefix = rospy.get_param('~robot_prefix', 'pr2') 
        self.srvroot = '/'+robot_prefix+'_'+whicharm+'_arm_kinematics/' 

        #If collision_aware_ik is set to 0, then collision-aware IK is disabled 
 	self.perception_running = rospy.get_param('~collision_aware_ik', 1) 

        self._ik_service = rospy.ServiceProxy(self.srvroot+'get_ik', GetPositionIK, True)
        if self.perception_running:
            self._ik_service_with_collision = rospy.ServiceProxy(self.srvroot+'get_constraint_aware_ik', GetConstraintAwarePositionIK, True)

        self._fk_service = rospy.ServiceProxy(self.srvroot+'get_fk', GetPositionFK, True)
        self._query_service = rospy.ServiceProxy(self.srvroot+'get_ik_solver_info', GetKinematicSolverInfo, True)
        self._check_state_validity_service = rospy.ServiceProxy('/planning_scene_validity_server/get_state_validity', GetStateValidity, True)

        #wait for IK/FK/query services and get the joint names and limits 
        if wait_for_services:
            self.check_services_and_get_ik_info()
        
        if tf_listener == None:
            rospy.loginfo("ik_utilities: starting up tf_listener")
            self.tf_listener = tf.TransformListener()
        else:
            self.tf_listener = tf_listener

        self.marker_pub = rospy.Publisher('interpolation_markers', Marker)

        #dictionary for the possible kinematics error codes
        self.error_code_dict = {}  #codes are things like SUCCESS, NO_IK_SOLUTION
        for element in dir(ArmNavigationErrorCodes):
            if element[0].isupper():
                self.error_code_dict[eval('ArmNavigationErrorCodes.'+element)] = element

        #reads the start angles from the parameter server 
        start_angles_list = rospy.get_param('~ik_start_angles', []) 
 		         
        #good additional start angles to try for IK for the PR2, used  
        #if no start angles were provided 
        if start_angles_list == []: 
            self.start_angles_list = [[-0.697, 1.002, 0.021, -0.574, 0.286, -0.095, 1.699], 
                                      [-1.027, 0.996, 0.034, -0.333, -3.541, -0.892, 1.694], 
                                      [0.031, -0.124, -2.105, -1.145, -1.227, -1.191, 2.690], 
                                      [0.410, 0.319, -2.231, -0.839, -2.751, -1.763, 5.494], 
                                      [0.045, 0.859, 0.059, -0.781, -1.579, -0.891, 7.707], 
                                      [0.420, 0.759, 0.014, -1.099, -3.204, -1.907, 8.753], 
                                      [-0.504, 1.297, -1.857, -1.553, -4.453, -1.308, 9.572]] 
        else: 
            self.start_angles_list = start_angles_list 

        if whicharm == 'left':
            for i in range(len(self.start_angles_list)):
                for joint_ind in [0, 2, 4]:
                    self.start_angles_list[i][joint_ind] *= -1.

        #changes the set of ids used to show the arrows every other call
        self.pose_id_set = 0

        rospy.loginfo("ik_utilities: done init")


    #wait for the various services and run the IK query to get the joint names and limits
    def check_services_and_get_ik_info(self):

        rospy.loginfo("ik_utilities: waiting for IK services to be there")
        rospy.wait_for_service(self.srvroot+'get_ik')
        if self.perception_running:
            rospy.wait_for_service(self.srvroot+'get_constraint_aware_ik')
        rospy.wait_for_service(self.srvroot+'get_fk')
        rospy.wait_for_service(self.srvroot+'get_ik_solver_info')
        rospy.loginfo("ik_utilities: services found")

        #get and store the joint names and limits
        #only one IK link available so far ('r_wrist_roll_link' or 'l_wrist_roll_link')
        rospy.loginfo("getting the IK solver info")
        (self.joint_names, self.min_limits, self.max_limits, self.link_names) = \
            self.run_query()
        self.link_name = self.link_names[-1]
        rospy.loginfo("done getting the IK solver info")


    ##draw a PoseStamped in rviz as a set of arrows (x=red, y=green, z=blue)
    #id is the id number for the x-arrow (y is id+1, z is id+2)
    def draw_pose(self, pose_stamped, id):
        marker = Marker()
        marker.header = pose_stamped.header
        marker.ns = "basic_shapes"
        marker.type = 0 #arrow
        marker.action = 0 #add
        marker.scale.x = 0.01
        marker.scale.y = 0.02
        marker.color.a = 1.0
        marker.lifetime = rospy.Duration(30.0)

        orientation = pose_stamped.pose.orientation
        quat = [orientation.x, orientation.y, orientation.z, orientation.w]
        mat = tf.transformations.quaternion_matrix(quat)
        start = [pose_stamped.pose.position.x, pose_stamped.pose.position.y, pose_stamped.pose.position.z]
        x_end = list(mat[:,0][0:3]*.05 + numpy.array(start))
        y_end = list(mat[:,1][0:3]*.05 + numpy.array(start))
        z_end = list(mat[:,2][0:3]*.05 + numpy.array(start))
        #print "start: %5.3f, %5.3f, %5.3f"%(pose_stamped.pose.position.x, pose_stamped.pose.position.y, pose_stamped.pose.position.z)
        #print "x_end:", pplist(x_end)
        #print "y_end:", pplist(y_end)
        #print "z_end:", pplist(z_end)
        marker.id = id
        marker.points = [pose_stamped.pose.position, Point(*x_end)]
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        self.marker_pub.publish(marker)
        marker.id = id+1
        marker.points = [pose_stamped.pose.position, Point(*y_end)]
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        self.marker_pub.publish(marker)
        marker.id = id+2
        marker.points = [pose_stamped.pose.position, Point(*z_end)]
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0
        self.marker_pub.publish(marker)


    ##get the joint names and limits, and the possible link names for running IK
    def run_query(self):
        try:
            resp = self._query_service()
        except rospy.ServiceException, e:
            rospy.logerr("GetKinematicSolverInfo service call failed! error msg: %s"%e)
            return (None, None, None)

        min_limits = [limit.min_position for limit in resp.kinematic_solver_info.limits]
        max_limits = [limit.max_position for limit in resp.kinematic_solver_info.limits]
#         rospy.loginfo("joint_names:" + str(resp.kinematic_solver_info.joint_names))
#         rospy.loginfo("min_limits:" + pplist(min_limits))
#         rospy.loginfo("max_limits:" + pplist(max_limits))
#         rospy.loginfo("link names:" + str(resp.kinematic_solver_info.link_names))
        return (resp.kinematic_solver_info.joint_names, min_limits, max_limits, resp.kinematic_solver_info.link_names)


    ##run forward kinematics on a set of 7 joint angles 
    #link_name specifies the desired output frame
    #returns a PoseStamped
    def run_fk(self, angles, link_name):
        if link_name not in self.link_names:
            rospy.logerr("link name %s not possible!"%link_name)
            return None
        self.link_name = link_name

        header = rospy.Header()
        header.stamp = rospy.get_rostime()
        header.frame_id = 'base_link'
        joint_state = JointState(header, self.joint_names, angles, [], [])
        try:
            resp = self._fk_service(header, [link_name], RobotState(joint_state, MultiDOFJointState())) 
        except rospy.ServiceException, e:
            rospy.logerr("FK service call failed! error msg: %s"%e)
            return None

        #ik failed, print the error code
        if resp.error_code.val != 1:
            rospy.loginfo("FK error code: %s"%self.error_code_dict[resp.error_code.val])

        return resp.pose_stamped[0]


    ##run inverse kinematics on a PoseStamped (7-dof pose 
    #(position + quaternion orientation) + header specifying the 
    #frame of the pose)
    #tries to stay close to double start_angles[7]
    #returns the solution angles as double solution[7] 
    #link_name is the link frame to position at pose
    #if collision_aware is 1, runs ik_service_with_collision
    #ordered_collision_operations is a list of collision operations to feed to IK to modify the collision space
    def run_ik(self, pose_stamped, start_angles, link_name, collision_aware = 1, ordered_collision_operations = None, IK_robot_state = None, link_padding = None):

        if link_name not in self.link_names:
            rospy.logerr("link name %s not possible!"%link_name)
            return None
        self.link_name = link_name

        #print "request:\n", pose_stamped

        ik_request = PositionIKRequest()
        ik_request.ik_link_name = self.link_name
        ik_request.pose_stamped = pose_stamped
        ik_request.ik_seed_state.joint_state.header.stamp = rospy.get_rostime()
        ik_request.ik_seed_state.joint_state.position = start_angles
        ik_request.ik_seed_state.joint_state.name = self.joint_names

        if IK_robot_state:
            ik_request.robot_state = IK_robot_state

        try:
            if collision_aware and self.perception_running:
                col_free_ik_request = GetConstraintAwarePositionIKRequest()
                col_free_ik_request.ik_request = ik_request
                col_free_ik_request.timeout = rospy.Duration(10.0) #timeout after 10 seconds
            
                if ordered_collision_operations != None:
                    col_free_ik_request.ordered_collision_operations = ordered_collision_operations
                if link_padding != None:
                    col_free_ik_request.link_padding = link_padding
                    
                resp = self._ik_service_with_collision(col_free_ik_request)
            else:
                resp = self._ik_service(ik_request, rospy.Duration(10.0))        
        except rospy.ServiceException, e:
            rospy.logwarn("IK service call failed! error msg: %s"%e)
            return (None, None)
        
        #ik failed, print the error code
        if resp.error_code.val != 1:
            rospy.loginfo("IK error code: %s"%self.error_code_dict[resp.error_code.val])
            #print "requested pose:\n", pose_stamped
        
        return (resp.solution.joint_state.position, self.error_code_dict[resp.error_code.val])


    ##check whether a set of joint angles is in collision with the environment
    #allows the same modifications as IK
    def check_state_validity(self, joint_angles, ordered_collision_operations = None, \
                                 robot_state = None, link_padding = None):
        
        req = GetStateValidityRequest()
        if robot_state != None:
            req.robot_state = robot_state
        req.robot_state.joint_state.name.extend(self.joint_names)
        req.robot_state.joint_state.position.extend(joint_angles)
        req.robot_state.joint_state.header.stamp = rospy.Time.now()
        req.check_collisions = True

        if ordered_collision_operations != None:
            req.ordered_collision_operations = ordered_collision_operations
        if link_padding != None:
            req.link_padding = link_padding

        try:
            res = self._check_state_validity_service(req)
        except rospy.ServiceException, e:
            rospy.logwarn("Check state validity call failed!  error msg: %s"%e)
            return 0
        if res.error_code.val == res.error_code.SUCCESS:
            return 1
        rospy.loginfo("Check state validity error code: %s"%self.error_code_dict[res.error_code.val])
        return 0


    ##convert a pointStamped to a pos list in a desired frame
    def point_stamped_to_list(self, point, frame):

        #convert the pointStamped to the desired frame, if necessary
        if point.header.frame_id != frame:
            try:
                trans_point = self.tf_listener.transformPoint(frame, point)
            except rospy.ServiceException, e:
                print "point:\n", point
                print "frame:", frame
                rospy.logerr("point_stamped_to_list: error in transforming point from " + point.header.frame_id + " to " + frame + "error msg: %s"%e)
                return None
        else:
            trans_point = point
        
        #extract position as a list
        pos = [trans_point.point.x, trans_point.point.y, trans_point.point.z]

        return pos


    ##convert a Vector3Stamped to a rot list in a desired frame
    def vector3_stamped_to_list(self, vector3, frame):

        #convert the vector3Stamped to the desired frame, if necessary
        if vector3.header.frame_id != frame:
            try:
                trans_vector3 = self.tf_listener.transformVector3(frame, vector3)
            except rospy.ServiceException, e:
                print "vector3:\n", vector3
                print "frame:", frame
                rospy.logerr("vector3_stamped_to_list: error in transforming point from " + vector3.header.frame_id + " to " + frame + "error msg: %s"%e)
                return None
        else:
            trans_vector3 = vector3
        
        #extract vect as a list
        vect = [trans_vector3.vector.x, trans_vector3.vector.y, trans_vector3.vector.z]

        return vect


    ##convert a QuaternionStamped to a quat list in a desired frame
    def quaternion_stamped_to_list(self, quaternion, frame):

        #convert the QuaternionStamped to the desired frame, if necessary
        if quaternion.header.frame_id != frame:
            try:
                trans_quat = self.tf_listener.transformQuaternion(frame, quaternion)
            except rospy.ServiceException, e:
                print "quaternion:\n", quaternion
                print "frame:", frame
                rospy.logerr("quaternion_stamped_to_list: error in transforming point from " + quaternion.header.frame_id + " to " + frame + "error msg: %s"%e)
                return None
        else:
            trans_quat = quaternion
        
        #extract quat as a list
        quat = [trans_quat.quaternion.x, trans_quat.quaternion.y, trans_quat.quaternion.z, trans_quat.quaternion.w]

        return quat


    ##convert a poseStamped to pos and rot (quaternion) lists in a desired frame
    def pose_stamped_to_lists(self, pose, frame):

        #convert the poseStamped to the desired frame, if necessary
        if pose.header.frame_id != frame:
            pose.header.stamp = rospy.Time(0)
            self.tf_listener.waitForTransform(frame, pose.header.frame_id, pose.header.stamp, rospy.Duration(5))
            try:
                trans_pose = self.tf_listener.transformPose(frame, pose)
            except rospy.ServiceException, e:
                print "pose:\n", pose
                print "frame:", frame
                rospy.logerr("pose_stamped_to_lists: error in transforming pose from " + pose.header.frame_id + " to " + frame + "error msg: %s"%e)
                return (None, None)
        else:
            trans_pose = pose
        
        #extract position and orientation as quaternion
        pos = [trans_pose.pose.position.x, trans_pose.pose.position.y, trans_pose.pose.position.z]
        rot = [trans_pose.pose.orientation.x, trans_pose.pose.orientation.y, \
                   trans_pose.pose.orientation.z, trans_pose.pose.orientation.w]

        return (pos, rot)


    ##convert pos and rot lists (relative to in_frame) to a poseStamped (relative to to_frame)
    def lists_to_pose_stamped(self, pos, rot, in_frame, to_frame):
        
        #stick lists in a poseStamped
        m = PoseStamped()
        m.header.frame_id = in_frame
        m.header.stamp = rospy.get_rostime()
        m.pose = Pose(Point(*pos), Quaternion(*rot))
        
        try:
            pose_stamped = self.tf_listener.transformPose(to_frame, m)
        except rospy.ServiceException, e:            
            rospy.logerr("error in transforming pose from " + in_frame + " to " + to_frame + "err msg: %s"%e)
            return None
        return pose_stamped


    ##vector norm of a list
    def vect_norm(self, vect):
        return sum([x**2 for x in vect])**.5


    ##normalize a vector
    def normalize_vect(self, vect):
        return list(numpy.array(vect)/self.vect_norm(vect))


    ##angle between two quaternions (as lists)
    def quat_angle(self, quat1, quat2):
        dot = sum([x*y for (x,y) in zip(quat1, quat2)])
        if dot > 1.:
            dot = 1.
        if dot < -1.:
            dot = -1.
        angle = 2*math.acos(math.fabs(dot))
        return angle


    ##interpolate a Cartesian path (expressed as pos and rot lists)
    #pos_spacing is max wrist translation in meters between trajectory points
    #rot_spacing is max wrist rotation in radians between trajectory points
    #num_steps overrides the number of steps (if != 0, ignore pos_spacing and rot_spacing)
    def interpolate_cartesian(self, start_pos, start_rot, end_pos, end_rot, pos_spacing, rot_spacing, num_steps = 0):

        #normalize quaternion rotations just in case
        norm_start_rot = self.normalize_vect(start_rot)
        norm_end_rot = self.normalize_vect(end_rot)

        #the desired wrist translation
        diff = [x-y for (x,y) in zip(end_pos, start_pos)]

        if num_steps == 0:
            #compute how far the wrist translates
            pos_move = self.vect_norm(diff)

            #compute how far the wrist rotates
            rot_move = self.quat_angle(norm_start_rot, norm_end_rot)

            #compute the number of steps to move no more than pos_spacing and rot_spacing in each step
            #(min 2, start and end)
            num_steps_pos = math.floor(pos_move/pos_spacing)+1
            num_steps_rot = math.floor(rot_move/rot_spacing)+1
            num_steps = int(max([num_steps_pos, num_steps_rot])+1)  
        
        #interpolate
        steps = []
        for stepind in range(num_steps):
            fraction = float(stepind)/(num_steps-1)  #add both start (0) and end (1)
            rot = list(tf.transformations.quaternion_slerp(norm_start_rot, norm_end_rot, fraction))
            pos = list(numpy.array(diff)*fraction + numpy.array(start_pos))
            steps.append((pos, rot))
            #print "fraction: %5.3f"%fraction, "pos:", pplist(pos), "rot:", pplist(rot)

        return steps
            
    
    ##check that all differences between angles1 and angles2 (lists of joint angles) are within consistent_range
    def check_consistent(self, angles1, angles2, consistent_range):
        diff = [math.fabs(x-y) for (x,y) in zip(angles1, angles2)]
        inconsistent = any([x>consistent_range for x in diff])
        return not inconsistent


    ##generate appropriate times and joint velocities for a joint path (such as that output by check_cartesian_path)
    #max_joint_vels is a list of maximum velocities to move the arm joints
    #max_joint_accs is a list of maximum accelerations to move the arm joints (can be ignored)
    #starts and ends in stop
    def trajectory_times_and_vels(self, joint_path, max_joint_vels = [.2]*7, max_joint_accs = [.5]*7):

        #min time for each segment
        min_segment_time = .01
        
        if not joint_path:
            rospy.logdebug("joint path was empty!")
            return([], [])
        traj_length = len(joint_path)
        num_joints = len(joint_path[0])

        #sanity-check max vels and accelerations
        if not max_joint_vels:
            max_joint_vels = [.2]*7
        elif len(max_joint_vels) != num_joints:
            rospy.logerr("invalid max_joint_vels!")
            return ([], [])
        if not max_joint_accs:
            max_joint_accs = [.5]*7
        elif len(max_joint_accs) != num_joints:
            rospy.logerr("invalid max_joint_accs!")
            return ([], [])
        for ind in range(num_joints):
            if max_joint_vels[ind] <= 0.:
                max_joint_vels[ind] = .2
            if max_joint_accs[ind] <= 0.:
                max_joint_accs[ind] = .5
            
        vels = [[None]*num_joints for i in range(traj_length)]
        
        #give the trajectory a bit of time to start
        segment_times = [None]*traj_length
        segment_times[0] = 0.05 

        #find vaguely appropriate segment times, assuming that we're traveling at max_joint_vels at the fastest joint
        for ind in range(traj_length-1):
            joint_diffs = [math.fabs(joint_path[ind+1][x]-joint_path[ind][x]) for x in range(num_joints)]
            joint_times = [diff/vel for (diff, vel) in zip(joint_diffs, max_joint_vels)]
            segment_times[ind+1] = max(joint_times+[min_segment_time])
            
        #set the initial and final velocities to 0 for all joints
        vels[0] = [0.]*num_joints
        vels[traj_length-1] = [0.]*num_joints

        #also set the velocity where any joint changes direction to be 0 for that joint
        #and otherwise use the average velocity (assuming piecewise-linear velocities for the segments before and after)
        for ind in range(1, traj_length-1):
            for joint in range(num_joints):
                diff0 = joint_path[ind][joint]-joint_path[ind-1][joint]
                diff1 = joint_path[ind+1][joint]-joint_path[ind][joint]
                if (diff0>0 and diff1<0) or (diff0<0 and diff1>0):
                    vels[ind][joint] = 0.
                else:
                    vel0 = diff0/segment_times[ind]
                    vel1 = diff1/segment_times[ind+1]
                    vels[ind][joint] = (vel0+vel1)/2.

        #increase the times if the desired velocities would require overly large accelerations
        for ind in range(1, traj_length):
            for joint in range(num_joints):
                veldiff = math.fabs(vels[ind][joint]-vels[ind-1][joint])
                acc = veldiff/segment_times[ind]
                try:
                    if acc > max_joint_accs[joint]:
                        segment_times[ind] = veldiff/max_joint_accs[joint]
                except:
                    pdb.set_trace()

        #turn the segment_times into waypoint times (cumulative)
        times = [None]*traj_length
        times[0] = segment_times[0]
        for ind in range(1, traj_length):
            try:
                times[ind] = times[ind-1]+segment_times[ind]
            except:
                pdb.set_trace()

        #return the times and velocities
        return (times, vels)

            
    ##check a Cartesian path for consistent, non-colliding IK solutions
    #start_pose and end_pose are PoseStamped messages with the wrist poses
    #start_angles are angles to try to stay close to
    #num_steps is the number of interpolation steps to use (if 0, use 
    #  pos_spacing and rot_spacing instead to calculate the number of steps)
    #pos_spacing is max wrist translation in meters between trajectory points 
    #rot_spacing is max wrist rotation in radians between trajectory points
    #consistent_angle is the max joint angle change before 2 steps are declared inconsistent
    #collision_check_resolution is the resolution at which to check collisions (0 or 1 for every)
    #steps_before_abort is the number of invalid steps found before aborting (-1 to ignore)
    #if collision_aware is 0, ignore collisions
    #ordered_collision_operations is an optional list of collision operations to feed to IK to modify the collision space
    #link_padding is an optional list of link paddings to feed to IK to modify the robot collision padding
    #IK_robot_state is an optional RobotState message to pass to IK
    #if start_from_end is 1, find an IK solution for the end first and work backwards
    #returns the joint angle trajectory and the error codes (0=good, 
    #  1=collisions, 2=inconsistent, 3=out of reach, 4=aborted before checking)
    def check_cartesian_path(self, start_pose, end_pose, start_angles, pos_spacing = 0.01, rot_spacing = 0.1, consistent_angle = math.pi/9., collision_aware = 1, collision_check_resolution = 1, steps_before_abort = -1, num_steps = 0, ordered_collision_operations = None, start_from_end = 0, IK_robot_state = None, link_padding = None, use_additional_start_angles = 0):

        #sanity-checking
        if num_steps != 0 and num_steps < 2:
            num_steps = 2
        if collision_check_resolution < 1:
            collision_check_resolution = 1
        if num_steps == 0 and (pos_spacing <= 0 or rot_spacing <= 0):
            rospy.logerr("invalid pos_spacing or rot_spacing")
            return ([], [])

        #convert to lists
        (start_pos, start_rot) = self.pose_stamped_to_lists(start_pose, 'base_link')
        (end_pos, end_rot) = self.pose_stamped_to_lists(end_pose, 'base_link')
        if start_pos == None or end_pos == None:
            return (None, None)
        
        #interpolate path
        steps = self.interpolate_cartesian(start_pos, start_rot, end_pos, end_rot, pos_spacing, rot_spacing, num_steps)

        #run collision-aware ik on each step, starting from the end and going backwards if start_from_end is true
        if start_from_end:
            steps.reverse()

        #use additional start angles from a pre-chosen set
        if use_additional_start_angles:
            num_to_use = max(use_additional_start_angles, len(self.start_angles_list))
            start_angles_list = [start_angles,] + self.start_angles_list[0:num_to_use]
        else:
            start_angles_list = [start_angles,]

        #go through each set of start angles, see if we can find a consistent trajectory
        for (start_angles_ind, start_angles) in enumerate(start_angles_list):
            trajectory = []
            error_codes = [] 
            
            if use_additional_start_angles:
                rospy.loginfo("start_angles_ind: %d"%start_angles_ind)

            for stepind in range(len(steps)):
                (pos,rot) = steps[stepind]
                pose_stamped = self.lists_to_pose_stamped(pos, rot, 'base_link', 'base_link')

                #draw the pose in rviz that we're checking in IK
                self.draw_pose(pose_stamped, stepind*3+self.pose_id_set*50)
                self.pose_id_set = (self.pose_id_set+1)%2

                #check for a non-collision_aware IK solution first
                (colliding_solution, error_code) = self.run_ik(pose_stamped, start_angles, self.link_name, collision_aware = 0, IK_robot_state = IK_robot_state)
                if not colliding_solution:
                    rospy.loginfo("non-collision-aware IK solution not found for step %d!"%stepind)
                    trajectory.append([0.]*7)
                    error_codes.append(3)          #3=out of reach

                else:
                    #if we're checking for collisions, then look for a collision-aware solution
                    collision_aware_this_step = collision_aware and (stepind % collision_check_resolution == 0 or stepind == len(steps)-1)
                    if collision_aware_this_step:
                        (solution, error_code) = self.run_ik(pose_stamped, start_angles, self.link_name, collision_aware, ordered_collision_operations, IK_robot_state = IK_robot_state, link_padding = link_padding)
                        if not solution:
                            rospy.loginfo("non-colliding IK solution not found for step %d!"%stepind)
                            collision_aware_solution_found = 0
                            solution = colliding_solution
                        else:
                            collision_aware_solution_found = 1
                    else:                
                        solution = colliding_solution

                    trajectory.append(list(solution))

                    #first trajectory point, or last point was all 0s, or consistent with previous point
                    if stepind == 0 or error_codes[-1] == 3 or self.check_consistent(trajectory[-2], solution, consistent_angle):
                        if not collision_aware_this_step or collision_aware_solution_found:
                            error_codes.append(0)  #0=good
                        else:
                            error_codes.append(1)  #1=collisions
                    else:
                        rospy.loginfo("IK solution not consistent for step %d!"%stepind)
                        error_codes.append(2)      #2=inconsistent

                    start_angles = solution

                #check if we should abort due to finding too many invalid points
                if error_codes[-1] > 0 and steps_before_abort >= 0 and stepind >= steps_before_abort:
                    rospy.loginfo("aborting due to too many invalid steps")
                    trajectory.extend([[0.]*7 for i in range(len(steps)-stepind-1)])
                    error_codes.extend([4]*(len(steps)-stepind-1)) #4=aborted before checking
                    break

            #if we didn't abort, stop and return the trajectory
            #if not any(error_codes):
            else:
                break

        if start_from_end:
            trajectory.reverse()        
            error_codes.reverse()
        return (trajectory, error_codes)


#test functions
if __name__ == '__main__':

    def keypause():
        print "press enter to continue"
        raw_input()

    #run check_cartesian_path after converting lists to poseStamped
    def check_cartesian_path_lists(ik_utilities, approachpos, approachquat, grasppos, graspquat, start_angles, pos_spacing = 0.01, \
                                       rot_spacing = 0.1, consistent_angle = math.pi/9., collision_aware = 1, \
                                       collision_check_resolution = 1, steps_before_abort = -1, num_steps = 0, frame = 'base_link'):

        print "approachpos:", pplist(approachpos), "  approachquat:", pplist(approachquat)
        print "grasppos:   ", pplist(grasppos), "  graspquat:   ", pplist(graspquat)

        start_pose = ik_utilities.lists_to_pose_stamped(approachpos, approachquat, frame, frame)
        end_pose = ik_utilities.lists_to_pose_stamped(grasppos, graspquat, frame, frame)

        (trajectory, error_codes) = ik_utilities.check_cartesian_path(start_pose, \
                      end_pose, start_angles, pos_spacing, rot_spacing, consistent_angle, collision_aware, collision_check_resolution, \
                      steps_before_abort, num_steps, use_additional_start_angles = 2)
        (times, vels) = ik_utilities.trajectory_times_and_vels(trajectory, [.2]*7, [.5]*7) 

        rospy.loginfo("trajectory:")
        for ind in range(len(trajectory)):
            rospy.loginfo("error code "+ str(error_codes[ind]) + " pos : " + pplist(trajectory[ind]))
        rospy.loginfo("")
        for ind in range(len(trajectory)):
            rospy.loginfo("time: " + "%5.3f  "%times[ind] + "vels: " + pplist(vels[ind]))

        return (trajectory, error_codes)


    #test check_cartesian_path
    def run_cartesian_path_test(ik_utilities):

        #side grasp
        #start_angles = [-0.447, -0.297, -2.229, -0.719, 0.734, -1.489, 1.355]
        start_angles = [0]*7
        sideapproachmat = numpy.array([[0., -1., 0., 0.],  
                                       [1., 0., 0., 0.],
                                       [0., 0., 1., 0.],
                                       [0., 0., 0., 1.]])
        sideapproachpos = [.62, -.3, .75]
        approachmat = sideapproachmat
        approachpos = sideapproachpos
        approachquat = list(tf.transformations.quaternion_from_matrix(approachmat))
        sidegrasppos = sideapproachpos[:]
        sidegrasppos[1] += .05
        grasppos = sidegrasppos
        graspquat = approachquat[:]
        print "side grasp"
        check_cartesian_path_lists(ik_utilities, approachpos, approachquat, grasppos, graspquat, start_angles)        


        #top to side grasp
        start_angles = [0.]*7
        approachpos = [.62, -.05, .85]
        approachquat = [-0.5, 0.5, 0.5, 0.5]  #from the top
        grasppos = [.62, -.05, .75]
        graspquat = [0.00000, 0.00000, 0.70711, 0.70711]  #from the side
        print "top to side grasp, collision-aware"
        check_cartesian_path_lists(ik_utilities, approachpos, approachquat, grasppos, graspquat, start_angles, pos_spacing = .01, rot_spacing = .2)

        print "more finely spaced rotations, not collision-aware"
        check_cartesian_path_lists(ik_utilities, approachpos, approachquat, grasppos, graspquat, start_angles, pos_spacing = .02, rot_spacing = .05, collision_aware = 0)

        print "5 steps, too far apart for default consistent_angle"
        check_cartesian_path_lists(ik_utilities, approachpos, approachquat, grasppos, graspquat, start_angles, num_steps = 5)

        print "5 steps, consistent_angle of pi/4"
        check_cartesian_path_lists(ik_utilities, approachpos, approachquat, grasppos, graspquat, start_angles, num_steps = 5, consistent_angle = math.pi/4)

        #top grasp through the table
        start_angles = [0.]*7
        approachpos = [.62, -.05, .65]
        approachquat = [-0.5, 0.5, 0.5, 0.5]  #from the top
        grasppos = [.62, -.05, .25]
        graspquat = approachquat[:]
        print "top grasp through the table"
        check_cartesian_path_lists(ik_utilities, approachpos, approachquat, grasppos, graspquat, start_angles, pos_spacing = 0.02, collision_aware = 1, collision_check_resolution = 2, steps_before_abort = -1)

        print "abort early"
        check_cartesian_path_lists(ik_utilities, approachpos, approachquat, grasppos, graspquat, start_angles, pos_spacing = 0.02, collision_aware = 1, collision_check_resolution = 2, steps_before_abort = 1)

        print "not collision-aware"
        check_cartesian_path_lists(ik_utilities, approachpos, approachquat, grasppos, graspquat, start_angles, pos_spacing = 0.02, collision_aware = 0, steps_before_abort = -1)

        print "12 steps, collision-aware, collision_check_resolution of 3"
        check_cartesian_path_lists(ik_utilities, approachpos, approachquat, grasppos, graspquat, start_angles, collision_aware = 1, collision_check_resolution = 3, num_steps = 12)


    #test ik and fk using a randomly-generated joint pose
    def run_ik_and_fk_test(ik_utilities, collision_aware = 1):

        #check if two lists of numbers are near each other
        trans_near = 1e-4
        rot_near = 1e-4
        def not_near(list1, list2, near):
            return any([math.fabs(v1-v2) > near for (v1, v2) in zip(list1, list2)])        

        #random seed based on the current time
        random.seed()

        #generate a random joint-angle configuration
        random_angles = [random.uniform(min, max) for (min, max) in \
                             zip(ik_utilities.min_limits, ik_utilities.max_limits)]

        #run forward kinematics to get the corresponding Cartesian pose
        fk_pose = ik_utilities.run_fk(random_angles, ik_utilities.link_name)
        pos1 = fk_pose.pose.position
        rot1 = fk_pose.pose.orientation
        pos1list = [pos1.x, pos1.y, pos1.z]
        rot1list = [rot1.x, rot1.y, rot1.z, rot1.w]

        #run inverse kinematics on that pose to get a set of angles
        #that also make that Cartesian pose (starting from all 0 angles)
        rospy.loginfo("IK request: pos "+pplist(pos1list)+" rot "+pplist(rot1list))
        (ik_angles, error_code) = ik_utilities.run_ik(fk_pose, [0]*7, ik_utilities.link_name, collision_aware)
        if not ik_angles:
            
            #test check_state_validity to see if it is in agreement about random_angles being in collision
            if collision_aware:
                valid = ik_utilities.check_state_validity(random_angles)
                if valid:
                    rospy.logerr("check state validity thinks random_angles is not in collision!")

            return 0

        #run forward kinematics on the ik solution to see if the
        #resulting pose matches the first
        fk_pose2 = ik_utilities.run_fk(ik_angles, ik_utilities.link_name)
        pos2 = fk_pose2.pose.position
        rot2 = fk_pose2.pose.orientation
        pos2list = [pos2.x, pos2.y, pos2.z]
        rot2list = [rot2.x, rot2.y, rot2.z, rot2.w]

        #test check_state_validity to see if it is in agreement about ik_angles not being in collision
        if collision_aware:
            valid = ik_utilities.check_state_validity(ik_angles)
            if not valid:
                rospy.logerr("check state validity thinks ik_angles is in collision!")

        #check the resulting solution 
        if not_near(pos1list, pos2list, trans_near) or \
           not_near(rot1list, rot2list, rot_near):
            rospy.loginfo("IK solution incorrect!")
            correct = 0
        else:
            rospy.loginfo("IK solution good")
            correct = 1
        rospy.loginfo("FK response:pos "+pplist(pos2list)+" rot "+pplist(rot2list))
        if not correct:
            return 0

        return 1


    def test_ik_with_specific_pose(ik_utilities):
        pose_stamped = ik_utilities.lists_to_pose_stamped([.62, -.05, .5], [.5, -.5, -.5, -.5], 'base_link', 'base_link')
        print "link name:", ik_utilities.link_name
        print "desired pose:\n", pose_stamped
        (solution, error_code) = ik_utilities.run_ik(pose_stamped, [0]*7, ik_utilities.link_name, 0)
        print "solution:", solution


    rospy.init_node('test_ik_and_fk', anonymous=True)
    
    ik_utilities = IKUtilities('right', wait_for_services = 0)
    ik_utilities.check_services_and_get_ik_info()

    print "testing ik and fk with random joint angles"
    for i in range(20):
       run_ik_and_fk_test(ik_utilities)

    print "testing the cartesian path interpolator"
    run_cartesian_path_test(ik_utilities)

    print "running a specific pose"
    test_ik_with_specific_pose(ik_utilities)





