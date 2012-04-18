#!/usr/bin/env python
import roslib; roslib.load_manifest('youbot_manipulation_examples')

import rospy
import actionlib
import tf
import arm_navigation_msgs.msg
import math


class MoveArmJointGoal:

	def __init__(self):
		self.action = actionlib.SimpleActionClient("/move_arm", arm_navigation_msgs.msg.MoveArmAction)
		self.action.wait_for_server()
		

	def move(self):
		goal = arm_navigation_msgs.msg.MoveArmGoal()
		goal.motion_plan_request.group_name = "arm"
		goal.motion_plan_request.num_planning_attempts = 1
		goal.motion_plan_request.planner_id = ""
		goal.planner_service_name = "/ompl_planning/plan_kinematic_path"
		goal.motion_plan_request.allowed_planning_time = rospy.Duration(60.0)
		
		joint_names = ["arm_joint_1", "arm_joint_2", "arm_joint_3", "arm_joint_4", "arm_joint_5"]
		
		for i in range(len(joint_names)):
			joint_constraint = arm_navigation_msgs.msg.JointConstraint()
			joint_constraint.joint_name = joint_names[i]
			joint_constraint.position = 0.1
			joint_constraint.tolerance_above = 0.1
			joint_constraint.tolerance_below = 0.1
			
			goal.motion_plan_request.goal_constraints.joint_constraints.append(joint_constraint)
		
		goal.motion_plan_request.goal_constraints.joint_constraints[2].position = -0.1
		
		self.action.send_goal(goal)
		return self.action.wait_for_result(rospy.Duration(200.0))


if __name__ == "__main__":
	rospy.init_node("move_arm_joint_goal")
	rospy.sleep(0.5)
	
	move = MoveArmJointGoal()
	print move.move()