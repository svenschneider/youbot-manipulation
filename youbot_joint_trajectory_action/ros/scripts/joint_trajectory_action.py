#!/usr/bin/env python
import roslib; roslib.load_manifest('youbot_joint_trajectory_action')

import rospy
import threading
import sensor_msgs.msg
import actionlib
import brics_actuator.msg
import control_msgs.msg


class JointTrajectoryAction:
	
	def __init__(self):
		self.lock = threading.Lock()
		self.received_state = False
		
		if (not rospy.has_param("joint_trajectory_action/unit")):
			rospy.logerr("No unit given.")
			exit(0)
		
		if (not rospy.has_param("joint_trajectory_action/joints")):
			rospy.logerr("No joints given.")
			exit(0)
		
		self.joint_names = rospy.get_param("joint_trajectory_action/joints")
		rospy.loginfo("Joints: %s", self.joint_names)
		self.configuration = [0 for i in range(len(self.joint_names))]
		
		self.unit = rospy.get_param("joint_trajectory_action/unit")
		rospy.loginfo("Unit: %s", self.unit)
		
		# subscriptions
		rospy.Subscriber('joint_states', sensor_msgs.msg.JointState, self.joint_states_callback)
		
		self.pub = rospy.Publisher('position_command', brics_actuator.msg.JointPositions)
		
		self.action = actionlib.SimpleActionServer("joint_trajectory_action", control_msgs.msg.FollowJointTrajectoryAction, execute_cb = self.execute_cb)
	
	
	def joint_states_callback(self, msg):
		self.lock.acquire()
		for k in range(len(self.joint_names)):
			for i in range(len(msg.name)):
				if (msg.name[i] == self.joint_names[k]):
					self.configuration[k] = msg.position[i]
		self.received_state = True
		self.lock.release()

	
	def execute_cb(self, goal):
		joint_positions = brics_actuator.msg.JointPositions()
		conf = goal.trajectory.points[0].positions
		
		# transform from ROS to BRICS message
		for i in range(len(self.joint_names)):
			joint_value = brics_actuator.msg.JointValue()
			joint_value.joint_uri = self.joint_names[i]
			joint_value.value = conf[i]
			joint_value.unit = self.unit
			joint_positions.positions.append(joint_value)
		self.pub.publish(joint_positions)
		
		# wait to reach the goal position
		is_timed_out = False
		start = rospy.Time.now()
		duration = rospy.Duration(5.0)
		while True:
			if (self.is_goal_reached(conf, self.configuration)):
				break
			if (rospy.Time.now() - start > duration):
				is_timed_out = True
				break
		
		result = control_msgs.msg.FollowJointTrajectoryResult()
		if (is_timed_out):
			result.error_code = control_msgs.msg.FollowJointTrajectoryResult.INVALID_GOAL
			self.action.set_aborted(result)
		else:
			result.error_code = control_msgs.msg.FollowJointTrajectoryResult.SUCCESSFUL
			self.action.set_succeeded(result)


	def is_goal_reached(self, goal, conf):
		for i in range(len(self.joint_names)):
			if (abs(goal[i] - conf[i]) > 0.05):
				return False
		return True


if __name__ == "__main__":
	rospy.init_node('joint_trajectory_action')
	rospy.sleep(0.5)
	
	action = JointTrajectoryAction()
	
	rospy.spin()