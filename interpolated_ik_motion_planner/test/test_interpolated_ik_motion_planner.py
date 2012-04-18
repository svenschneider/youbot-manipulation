#!/usr/bin/python
'''test client for interpolated_ik_motion_planner'''

import roslib
roslib.load_manifest('interpolated_ik_motion_planner')
import rospy
from arm_navigation_msgs.srv import GetMotionPlanRequest, GetMotionPlanResponse, GetMotionPlan
from arm_navigation_msgs.msg import PositionConstraint, OrientationConstraint, ArmNavigationErrorCodes, OrderedCollisionOperations, CollisionOperation
from geometry_msgs.msg import PoseStamped, PointStamped, QuaternionStamped, Pose, Point, Quaternion
from interpolated_ik_motion_planner.srv import SetInterpolatedIKMotionPlanParams
import math
import tf
import numpy
import pdb


#pretty-print list to string
def pplist(list):
    return ' '.join(['%5.3f'%x for x in list])


def keypause():
    print "press enter to continue"
    raw_input()


#call the service
def call_get_interpolated_ik_motion_plan(start_pose, goal_pose, start_angles, joint_names, pos_spacing = 0.01, \
                         rot_spacing = 0.1, consistent_angle = math.pi/9, collision_aware = 1, \
                         collision_check_resolution = 1, steps_before_abort = -1, num_steps = 0, \
                         ordered_collision_operations = None, frame = 'torso_lift_link', start_from_end = 0, \
                         max_joint_vels = [.2]*7, max_joint_accs = [.5]*7):
    print "waiting for service"
    rospy.wait_for_service("r_interpolated_ik_motion_plan")
    print "service found"

    try:
        serv = rospy.ServiceProxy("r_interpolated_ik_motion_plan_set_params", SetInterpolatedIKMotionPlanParams)
        res = serv(num_steps, consistent_angle, collision_check_resolution, steps_before_abort, pos_spacing, rot_spacing, collision_aware, start_from_end, max_joint_vels, max_joint_accs)
    except rospy.ServiceException, e:
        print "error when calling r_interpolated_ik_motion_plan_set_params: %s"%e  
        return 0        

    req = GetMotionPlanRequest()
    req.motion_plan_request.start_state.joint_state.name = joint_names
    req.motion_plan_request.start_state.joint_state.position = start_angles
    req.motion_plan_request.start_state.multi_dof_joint_state.pose = start_pose.pose
    req.motion_plan_request.start_state.multi_dof_joint_state.child_frame_id = 'r_wrist_roll_link'
    req.motion_plan_request.start_state.multi_dof_joint_state.frame_id = start_pose.header.frame_id
    pos_constraint = PositionConstraint()
    pos_constraint.position = goal_pose.pose.position
    pos_constraint.header.frame_id = goal_pose.header.frame_id
    req.motion_plan_request.goal_constraints.position_constraints = [pos_constraint,]

    orient_constraint = OrientationConstraint()
    orient_constraint.orientation = goal_pose.pose.orientation
    orient_constraint.header.frame_id = goal_pose.header.frame_id
    req.motion_plan_request.goal_constraints.orientation_constraints = [orient_constraint,]

    if ordered_collision_operations != None:
        req.motion_plan_request.ordered_collision_operations = ordered_collision_operations

    try:    
        serv = rospy.ServiceProxy("r_interpolated_ik_motion_plan", GetMotionPlan)
        res = serv(req)
    except rospy.ServiceException, e:
        print "error when calling r_interpolated_ik_motion_plan: %s"%e  
        return 0

    error_code_dict = {ArmNavigationErrorCodes.SUCCESS:0, ArmNavigationErrorCodes.COLLISION_CONSTRAINTS_VIOLATED:1, \
                       ArmNavigationErrorCodes.PATH_CONSTRAINTS_VIOLATED:2, ArmNavigationErrorCodes.JOINT_LIMITS_VIOLATED:3, \
                       ArmNavigationErrorCodes.PLANNING_FAILED:4}


    trajectory_len = len(res.trajectory.joint_trajectory.points)
    trajectory = [res.trajectory.joint_trajectory.points[i].positions for i in range(trajectory_len)]
    vels = [res.trajectory.joint_trajectory.points[i].velocities for i in range(trajectory_len)]
    times = [res.trajectory.joint_trajectory.points[i].time_from_start for i in range(trajectory_len)]
    error_codes = [error_code_dict[error_code.val] for error_code in res.trajectory_error_codes]

    rospy.loginfo("trajectory:")
    for ind in range(len(trajectory)):
        rospy.loginfo("error code "+ str(error_codes[ind]) + " pos : " + pplist(trajectory[ind]))
    rospy.loginfo("")
    for ind in range(len(trajectory)):
        rospy.loginfo("time: " + "%5.3f  "%times[ind].to_sec() + "vels: " + pplist(vels[ind]))


#create a PoseStamped message
def create_pose_stamped(pose, frame_id = 'torso_lift_link'):
    m = PoseStamped()
    m.header.frame_id = frame_id
    m.header.stamp = rospy.Time()
    m.pose = Pose(Point(*pose[0:3]), Quaternion(*pose[3:7]))
    return m


#convert to PoseStamped then call the service
def check_cartesian_path_lists(approachpos, approachquat, grasppos, graspquat, start_angles, pos_spacing = 0.01, \
                              rot_spacing = 0.1, consistent_angle = math.pi/9., collision_aware = 1, \
                              collision_check_resolution = 1, steps_before_abort = -1, num_steps = 0, \
                              ordered_collision_operations = None, frame = 'torso_lift_link'):
    joint_names = ["r_shoulder_pan_joint",
                    "r_shoulder_lift_joint",
                    "r_upper_arm_roll_joint",
                    "r_elbow_flex_joint",
                    "r_forearm_roll_joint",
                    "r_wrist_flex_joint",
                    "r_wrist_roll_joint"]
    start_pose = create_pose_stamped(approachpos+approachquat)
    goal_pose = create_pose_stamped(grasppos+graspquat)
    call_get_interpolated_ik_motion_plan(start_pose, goal_pose, start_angles, joint_names, pos_spacing, rot_spacing, \
                                             consistent_angle, collision_aware, collision_check_resolution, \
                                             steps_before_abort, num_steps, ordered_collision_operations)


if __name__ == "__main__":

    #side grasp
    #start_angles = [-0.447, -0.297, -2.229, -0.719, 0.734, -1.489, 1.355]
    start_angles = [0]*7
    sideapproachmat = numpy.array([[0., -1., 0., 0.],  
                                   [1., 0., 0., 0.],
                                   [0., 0., 1., 0.],
                                   [0., 0., 0., 1.]])
    sideapproachpos = [.67, -.3, .01]
    approachmat = sideapproachmat
    approachpos = sideapproachpos
    approachquat = list(tf.transformations.quaternion_from_matrix(approachmat))
    sidegrasppos = sideapproachpos[:]
    sidegrasppos[1] += .05
    grasppos = sidegrasppos
    graspquat = approachquat[:]
    print "side grasp"
    check_cartesian_path_lists(approachpos, approachquat, grasppos, graspquat, start_angles)        


    #top to side grasp
    start_angles = [0.]*7
    approachpos = [.72, -.05, .11]
    approachquat = [-0.5, 0.5, 0.5, 0.5]  #from the top
    grasppos = [.67, -.05, .01]
    graspquat = [0.00000, 0.00000, 0.70711, 0.70711]  #from the side
    print "top to side grasp, collision-aware"
    check_cartesian_path_lists(approachpos, approachquat, grasppos, graspquat, start_angles, pos_spacing = .01, rot_spacing = .2)

    print "more finely spaced rotations, not collision-aware"
    check_cartesian_path_lists(approachpos, approachquat, grasppos, graspquat, start_angles, pos_spacing = .02, rot_spacing = .05, collision_aware = 0)

    print "3 steps, too far apart for default consistent_angle"
    check_cartesian_path_lists(approachpos, approachquat, grasppos, graspquat, start_angles, num_steps = 3)

    print "3 steps, consistent_angle of pi/4"
    check_cartesian_path_lists(approachpos, approachquat, grasppos, graspquat, start_angles, num_steps = 3, consistent_angle = math.pi/4)


    #top grasp through the table (if it's there)
    start_angles = [0.]*7
    approachpos = [.67, -.05, .01]
    approachquat = [-0.5, 0.5, 0.5, 0.5]  #from the top
    grasppos = [.67, -.05, -.5]
    graspquat = approachquat[:]
    print "top grasp through the table"
    check_cartesian_path_lists(approachpos, approachquat, grasppos, graspquat, start_angles, pos_spacing = 0.02, collision_aware = 1, collision_check_resolution = 1, steps_before_abort = -1)

    print "using current start angles"
    check_cartesian_path_lists(approachpos, approachquat, grasppos, graspquat, start_angles = [], pos_spacing = 0.02, collision_aware = 1, collision_check_resolution = 1, steps_before_abort = -1)

    print "ignoring collisions with all collision points"
    collision_oper = CollisionOperation(object1 = "points", \
                                        object2 = CollisionOperation.COLLISION_SET_ALL, \
                                        operation = CollisionOperation.DISABLE)
    ordered_collision_operations = OrderedCollisionOperations([collision_oper,])
    check_cartesian_path_lists(approachpos, approachquat, grasppos, graspquat, start_angles, pos_spacing = 0.02, collision_aware = 1, collision_check_resolution = 1, steps_before_abort = -1, ordered_collision_operations = ordered_collision_operations)

    print "abort early"
    check_cartesian_path_lists(approachpos, approachquat, grasppos, graspquat, start_angles, pos_spacing = 0.02, collision_aware = 1, collision_check_resolution = 1, steps_before_abort = 1)

    print "not collision-aware"
    check_cartesian_path_lists(approachpos, approachquat, grasppos, graspquat, start_angles, pos_spacing = 0.02, collision_aware = 0, steps_before_abort = -1)

    print "12 steps, collision-aware, collision_check_resolution of 3"
    check_cartesian_path_lists(approachpos, approachquat, grasppos, graspquat, start_angles, collision_aware = 1, collision_check_resolution = 3, num_steps = 12)
    


