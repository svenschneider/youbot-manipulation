#!/usr/bin/env python
import roslib; roslib.load_manifest('youbot_manipulation_examples')

import rospy
import threading
import tf
import time
import math
import geometry_msgs.msg
import kinematics_msgs.srv
import kinematics_msgs.msg
import sensor_msgs.msg
import arm_navigation_msgs.msg
import arm_navigation_msgs.srv
import brics_actuator.msg

class SimpleIkSolver:

	def __init__(self):
		self.joint_names = ["arm_joint_1", "arm_joint_2", "arm_joint_3", "arm_joint_4", "arm_joint_5"]
		self.configuration = [0, 0, 0, 0, 0]
		self.received_state = False
		
		rospy.Subscriber('/joint_states', sensor_msgs.msg.JointState, self.joint_states_callback)
		
		rospy.loginfo("Waiting for 'get_constraint_aware_ik' service")
		rospy.wait_for_service('/youbot_arm_kinematics/get_constraint_aware_ik')
		self.ciks = rospy.ServiceProxy('/youbot_arm_kinematics/get_constraint_aware_ik', kinematics_msgs.srv.GetConstraintAwarePositionIK)
		rospy.loginfo("Service 'get_constraint_aware_ik' is ready")
		
		rospy.loginfo("Waiting for 'set_planning_scene_diff' service")
		rospy.wait_for_service('/environment_server/set_planning_scene_diff')
		self.planning_scene = rospy.ServiceProxy('/environment_server/set_planning_scene_diff', arm_navigation_msgs.srv.SetPlanningSceneDiff)
		rospy.loginfo("Service 'set_planning_scene_diff'")
		
		# a planning scene must be set before using the constraint-aware ik!
		self.send_planning_scene()


    #callback function: when a joint_states message arrives, save the values
	def joint_states_callback(self, msg):
		for k in range(5):
			for i in range(len(msg.name)):
				joint_name = "arm_joint_" + str(k + 1)
				if(msg.name[i] == joint_name):
					self.configuration[k] = msg.position[i]
		self.received_state = True


	def send_planning_scene(self):
		rospy.loginfo("Sending planning scene")
		
		req = arm_navigation_msgs.srv.SetPlanningSceneDiffRequest()
		res = self.planning_scene.call(req)


	def call_constraint_aware_ik_solver(self, goal_pose):
		while (not self.received_state):
			time.sleep(0.1)
		req = kinematics_msgs.srv.GetConstraintAwarePositionIKRequest()
		req.timeout = rospy.Duration(0.5)
		req.ik_request.ik_link_name = "arm_link_5"
		req.ik_request.ik_seed_state.joint_state.name = self.joint_names
		req.ik_request.ik_seed_state.joint_state.position = self.configuration
		req.ik_request.pose_stamped = goal_pose
		try:
			resp = self.ciks(req)
		except rospy.ServiceException, e:
			rospy.logerr("Service did not process request: %s", str(e))
		
		if (resp.error_code.val == arm_navigation_msgs.msg.ArmNavigationErrorCodes.SUCCESS):
			return resp.solution.joint_state.position
		else:
			return None


	def create_pose(self, x, y, z, roll, pitch, yaw):
		pose = geometry_msgs.msg.PoseStamped()
		pose.pose.position.x = x
		pose.pose.position.y = y
		pose.pose.position.z = z
		quat = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
		pose.pose.orientation.x = quat[0]
		pose.pose.orientation.y = quat[1]
		pose.pose.orientation.z = quat[2]
		pose.pose.orientation.w = quat[3]
		pose.header.frame_id = "/arm_link_0"
		pose.header.stamp = rospy.Time.now()
		
		return pose



if __name__ == "__main__":
	rospy.init_node("simple_ik_solver")
	time.sleep(0.5)
	
	armpub = rospy.Publisher("/arm_1/arm_controller/position_command", brics_actuator.msg.JointPositions)
	iks = SimpleIkSolver()
	
	x = 0.024 + 0.033 + 0.4
	y = 0.0
	z = 0.115
	roll = 0.0
	pitch = math.pi / 2.0
	yaw = 0.0
	
	pose = iks.create_pose(x, y, z, roll, pitch, yaw)
	
	conf = iks.call_constraint_aware_ik_solver(pose)
	if (conf):
		# publish solution directly as joint positions
		print conf
		jp = brics_actuator.msg.JointPositions()
		
		for i in range(5):
			jv = brics_actuator.msg.JointValue()
			jv.joint_uri = iks.joint_names[i]
			jv.value = conf[i]
			jv.unit = "rad"
			jp.positions.append(jv)
		
		rospy.sleep(0.5)
		print "publishing cmd"
		armpub.publish(jp)
	else:
		print("IK solver didn't find a solution")