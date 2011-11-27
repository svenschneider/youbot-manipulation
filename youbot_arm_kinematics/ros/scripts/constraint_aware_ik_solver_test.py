#!/usr/bin/env python
import roslib; roslib.load_manifest('youbot_arm_kinematics')

import rospy
import threading
import tf
import time
import math
import geometry_msgs.msg
import kinematics_msgs.srv
import kinematics_msgs.msg
import sensor_msgs.msg
import motion_planning_msgs.msg


class KinematicsTest:

	def __init__(self):
		self.joint_names = ["arm_joint_1", "arm_joint_2", "arm_joint_3", "arm_joint_4", "arm_joint_5"]
		self.configuration = [0, 0, 0, 0, 0]
		self.received_state = False

		rospy.Subscriber('/joint_states', sensor_msgs.msg.JointState, self.joint_states_callback)
		
		rospy.loginfo("Waiting for 'get_fk' service")
		rospy.wait_for_service("/youbot_arm_kinematics/get_fk")
		self.fk_solver = rospy.ServiceProxy("/youbot_arm_kinematics/get_fk", kinematics_msgs.srv.GetPositionFK)
		rospy.loginfo("Service 'get_fk' is ready")

		rospy.loginfo("Waiting for 'get_ik' service")
		rospy.wait_for_service('/youbot_arm_kinematics/get_ik')
		self.iks = rospy.ServiceProxy('/youbot_arm_kinematics/get_ik', kinematics_msgs.srv.GetPositionIK)
		rospy.loginfo("Service 'get_ik' is ready")

		rospy.loginfo("Waiting for 'get_constraint_aware_ik' service")
		rospy.wait_for_service('/youbot_arm_kinematics/get_constraint_aware_ik')
		self.ciks = rospy.ServiceProxy('/youbot_arm_kinematics/get_constraint_aware_ik', kinematics_msgs.srv.GetConstraintAwarePositionIK)
		rospy.loginfo("Service 'get_constraint_aware_ik' is ready")


	#callback function: when a joint_states message arrives, save the values
	def joint_states_callback(self, msg):
		for k in range(5):
			for i in range(len(msg.name)):
				joint_name = "arm_joint_" + str(k + 1)
				if(msg.name[i] == joint_name):
					self.configuration[k] = msg.position[i]
		self.received_state = True


	def call_fk_solver(self, configuration):
		while(not self.received_state):
			time.sleep(0.1)
		
		req = kinematics_msgs.srv.GetPositionFKRequest()
		req.header.frame_id = "base_link"
		req.header.stamp = rospy.Time.now()
		req.fk_link_names.append("arm_link_5")
		req.robot_state.joint_state.name = self.joint_names
		req.robot_state.joint_state.position = configuration
		try:
			resp = self.fk_solver(req)
		except rospy.ServiceException, e:
			rospy.logerr("Service did not process request: %s", str(e))
		
		if (resp.error_code.val == motion_planning_msgs.msg.ArmNavigationErrorCodes.SUCCESS):
			return (resp.pose_stamped[0], True)
		else:
			return (geometry_msgs.msg.PoseStamped(), False)


	def call_ik_solver(self, goal_pose):
		while(not self.received_state):
			time.sleep(0.1)
		req = kinematics_msgs.srv.GetPositionIKRequest()
		req.ik_request.ik_link_name = "arm_link_5"
		req.ik_request.ik_seed_state.joint_state.name = self.joint_names
		req.ik_request.ik_seed_state.joint_state.position = self.configuration
		req.ik_request.pose_stamped = goal_pose
		try:
			resp = self.iks(req)
		except rospy.ServiceException, e:
			rospy.logerr("Service did not process request: %s", str(e))
		return (resp.solution.joint_state.position, resp.error_code.val == motion_planning_msgs.msg.ArmNavigationErrorCodes.SUCCESS)


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
		return (resp.solution.joint_state.position, resp.error_code.val == motion_planning_msgs.msg.ArmNavigationErrorCodes.SUCCESS)



if __name__ == "__main__":
	rospy.init_node('youbot_ik_solver_test')
	time.sleep(0.5)
	
	iks = KinematicsTest()
	
	'''
	# Pointing upwards (internal home position of inverse kinematics)
	x = 0.024 + 0.033
	y = 0
	z = 0.535
	roll = 0
	pitch = 0
	yaw = 0
	'''
	
	'''
	# Pointing upwards on left
	x = 0.024
	y = 0.033
	z = 0.535
	roll = 0
	pitch = 0
	yaw = 0
	'''
	
	'''
	# Pointing to front
	x = 0.024 + 0.033 + 0.4
	y = 0
	z = 0.115
	roll = 0
	pitch = math.pi / 2.0
	yaw = 0
	'''
	
	'''
	# Pointing to left
	x = 0.024
	y = 0.033 + 0.4
	z = 0.115
	roll = 0
	pitch = math.pi / 2.0
	yaw = math.pi / 2.0
	'''
	
	'''
	# Pointing to right
	x = 0.024
	y = -0.033 - 0.4
	z = 0.115
	roll = 0
	pitch = math.pi / 2.0
	yaw = -math.pi / 2.0
	'''
	
	'''
	# Grasp from floor in front
	x =  0.25
	y =  0.0
	z = -0.05
	roll = 0
	pitch = math.pi
	yaw = 0
	'''
	
	'''
	# Grasp from floor in front with rotated last joint
	x =  0.25
	y =  0.0
	z = -0.05
	roll = 0
	pitch = math.pi
	yaw = math.pi / 4.0
	'''
	
	'''
	# Grasp from floor on right
	x =  0.024
	y = -0.25
	z = -0.05
	roll = 0
	pitch = math.pi
	yaw = 0
	'''
	
	'''
	# Pointing down on left
	x = 0.024
	y = 0.033 + 0.4 - 0.13
	z = 0
	roll = math.pi
	pitch = 0
	yaw = 0
	'''
	
	'''
	# Grasp from floor in left front
	x =  0.2
	y =  0.2
	z = -0.05
	roll = 0
	pitch = math.pi
	yaw = math.pi / 2.0
	'''
	
	'''
	# Grasp from floor in left front (gripper aligned with arm)
	x =  0.2
	y =  0.2
	z = -0.05
	roll = 0
	pitch = math.pi
	yaw = math.pi / 4.0
	'''
	
	'''
	# Grasp from floor in right front
	x =  0.2
	y = -0.2
	z = -0.05
	roll = 0
	pitch = math.pi
	yaw = math.pi / 2.0
	'''
	
	'''
	# Pointing slightly upwards in front (works)
	x = 0.024 + 0.033 + 0.3
	y = 0
	z = 0.115
	roll = 0
	pitch = math.pi / 2.0 - 0.56
	yaw = 0
	'''
	
	'''
	# Pointing slightly upwards in front (must fail!!) [This tests the redundancy of joint 3]
	x = 0.024 + 0.033 + 0.3
	y = 0
	z = 0.115
	roll = 0
	pitch = math.pi / 2.0 - 0.57
	yaw = 0
	'''
	
	'''
	# Pointing to rear left
	x = -0.2
	y =  0.033 + 0.2
	z =  0.115
	roll = 0
	pitch = math.pi / 2.0
	yaw = math.pi / 2.0
	'''
	
	'''
	# Pointing to rear right
	x = -0.2
	y = -0.033 - 0.2
	z =  0.115
	roll = 0
	pitch = math.pi / 2.0
	yaw = -math.pi / 2.0
	'''
	
	'''
	# Parallel over loading bay
	x = 0.033 + 0.024 - 0.35
	y = 0
	z = 0.08
	roll = 0
	pitch = -math.pi / 2.0
	yaw = 0
	'''
	
	'''
	# Grasping from far end of loading bay
	x = 0.033 + 0.024 - 0.38
	y = 0
	z = 0.08
	roll = 0
	pitch = -math.pi / 2.0 - 0.5
	yaw = 0
	'''
	
	'''
	# Grasping from near end of loading bay
	x = 0.033 + 0.024 - 0.25
	y = 0
	z = 0.115
	roll = 0
	pitch = -math.pi + 0.02
	yaw = 0
	'''
	
	'''
	# Pointing to rear left after flipping first joint
	x = -0.3
	y =  0.06
	z =  0.115
	roll = 0
	pitch = math.pi / 2.0
	yaw = math.pi / 2.0 + math.pi / 4.0
	'''
	
	'''
	# Pointing to rear left before flipping first joint
	x = -0.3
	y =  0.07
	z =  0.115
	roll = 0
	pitch = math.pi / 2.0
	yaw = math.pi / 2.0 + math.pi / 4.0
	'''
	
	'''
	# Grasp from floor on right side
	x =  0.0
	y = -0.25
	z = -0.05
	roll = 0
	pitch = math.pi
	yaw = math.pi / 2.0
	'''
	
	'''
	# Grasp from floor on right rear
	x = -0.15
	y = -0.25
	z = -0.05
	roll = 0
	pitch = math.pi
	yaw = 0
	'''
	
	'''
	# Pointing up in left front
	x = 0.024 + 0.033 + 0.2
	y = 0.1
	z = 0.4
	roll = 0
	pitch = 0
	yaw = 0
	'''
	
	
	# Pointing upwards (internal home position of inverse kinematics)
	x = 0.024 + 0.033
	y = 0
	z = 0.535
	roll = 0
	pitch = 0
	yaw = 0
	
	
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
	
	# (conf, success) = iks.call_ik_solver(pose)
	(conf, success) = iks.call_constraint_aware_ik_solver(pose)
	if (success):
		print(conf)
	else:
		print("IK solver didn't find a solution")