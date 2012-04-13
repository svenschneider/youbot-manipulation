#!/usr/bin/env python
import roslib; roslib.load_manifest('youbot_arm_kinematics')

import rospy
import actionlib
import tf
import time
import math
import random
import geometry_msgs.msg
import kinematics_msgs.srv
import kinematics_msgs.msg
import sensor_msgs.msg
import arm_navigation_msgs.msg
import arm_navigation_msgs.srv
import control_msgs.msg
import trajectory_msgs.msg


class MoveArm:

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
		
		
		self.arm_trajectory = actionlib.SimpleActionClient("/arm_1/arm_controller/joint_trajectory_action", control_msgs.msg.FollowJointTrajectoryAction)
		
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
	

	def move_to_pose(self, goal_pose):
		conf = self.call_constraint_aware_ik_solver(goal_pose)
		
		if (not conf):
			rospy.logerr("No inverse kinematics solution found")
			return False
		
		# trajectory point where the motion starts
		point_start = trajectory_msgs.msg.JointTrajectoryPoint()
		point_start.positions = conf
		point_start.time_from_start = rospy.Time(0.0)
		
		# trajectory point where the motion ends
		point_end = trajectory_msgs.msg.JointTrajectoryPoint()
		point_end.positions = conf
		point_start.time_from_start = rospy.Time(1.0)
		
		# set up the goal consisting of start and end configuration
		goal = control_msgs.msg.FollowJointTrajectoryGoal()
		goal.trajectory.joint_names = self.joint_names
		goal.trajectory.points.append(point_start)
		goal.trajectory.points.append(point_end)
		
		self.arm_trajectory.send_goal(goal)
		finished_before_timeout = self.arm_trajectory.wait_for_result(rospy.Duration(5.0))
		
		if (finished_before_timeout):
			res = self.arm_trajectory.get_state()
			type(res)
			
			if (res):
				rospy.loginfo("Reached the goal")
				return True
			else:
				rospy.logerr("Didn't reach the goal")
				return False
		else:
			rospy.logerr("Moving the arm to the goal failed due to a timeout")
			return False


if __name__ == "__main__":
	rospy.init_node('youbot_move_arm_cart_test')
	time.sleep(0.5)
	
	move = MoveArm()
	
	pose_list = []
	pose_list.append([0.024 + 0.033, 0.0, 0.535, 0.0, 0.0, 0.0])								# Pointing upwards (internal home position of inverse kinematics)
	pose_list.append([0.024, 0.033, 0.535, 0.0, 0.0, 0.0])										# Pointing upwards on left
	pose_list.append([0.024 + 0.033 + 0.4, 0.0, 0.115, 0, math.pi / 2.0, 0.0])					# Pointing to front
	pose_list.append([0.024, 0.033 + 0.3, 0.115, 0, math.pi / 2.0, math.pi / 2.0])				# Pointing to left
	pose_list.append([0.024, -0.033 - 0.3, 0.115, 0, math.pi / 2.0, -math.pi / 2.0])			# Pointing to right
	pose_list.append([0.25, 0.0, -0.05, 0, math.pi, 0.0])										# Grasp from floor in front
	pose_list.append([0.25, 0.0, -0.05, 0.0, math.pi, math.pi / 4.0])							# Grasp from floor in front with rotated last joint
	pose_list.append([0.2, 0.2, -0.05, 0.0, math.pi, math.pi / 2.0])							# Grasp from floor in left front
	pose_list.append([0.2, 0.2, -0.05, 0.0, math.pi, math.pi / 4.0])							# Grasp from floor in left front (gripper aligned with arm)
	pose_list.append([0.2, -0.2, -0.05, 0.0, math.pi, math.pi / 2.0])							# Grasp from floor in right front
	pose_list.append([-0.2, 0.033 + 0.2, 0.115, 0.0, math.pi / 2.0, math.pi / 2.0])				# Pointing to rear left
	pose_list.append([-0.2, -0.033 - 0.2, 0.115, 0.0, math.pi / 2.0, -math.pi / 2.0])			# Pointing to rear right
	pose_list.append([0.033 + 0.024 - 0.35, 0.0, 0.08, 0.0, -math.pi / 2.0, 0.0])				# Parallel over loading bay
	pose_list.append([0.033 + 0.024 - 0.38, 0.0, 0.08, 0.0, -math.pi / 2.0 - 0.5, 0.0])			# Grasping from far end of loading bay
	pose_list.append([0.033 + 0.024 - 0.25, 0.0, 0.115, 0.0, -math.pi + 0.02, 0.0])				# Grasping from near end of loading bay
	pose_list.append([-0.3, 0.06, 0.115, 0.0, math.pi / 2.0, math.pi / 2.0 + math.pi / 4.0])	# Pointing to rear left after flipping first joint
	pose_list.append([0.3, 0.07, 0.115, 0.0, math.pi / 2.0, math.pi / 2.0 + math.pi / 4.0])		# Pointing to rear left before flipping first joint
	pose_list.append([0.0, -0.25, -0.05, 0.0, math.pi, math.pi / 2.0])							# Grasp from floor on right side
	pose_list.append([-0.15, -0.25, -0.05, 0.0, math.pi, 0.0])									# Grasp from floor on right rear
	pose_list.append([0.024 + 0.033 + 0.2, 0.1, 0.4, 0.0, 0.0, 0.0])							# Pointing up in left front

	
	while (not rospy.is_shutdown()):
		i = random.randint(0, len(pose_list) - 1)
		
		pose = pose_list[i]
		ros_pose = move.create_pose(pose[0], pose[1], pose[2], pose[3], pose[4], pose[5])
		move.move_to_pose(ros_pose)