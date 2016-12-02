#!/usr/bin/env python

# Code based on Evan Sonderegger's code on https://github.com/esonderegger/youbot_srt


import roslib
import rospy
from brics_actuator.msg import JointPositions
from brics_actuator.msg import JointValue
from brics_actuator.msg import Poison
# from control_msgs.msg import FollowJointTrajectoryAction
from control_msgs.msg import GripperCommandAction
# from sensor_msgs.msg import JointState
from actionlib import SimpleActionServer
# import yaml
# import datetime

def nonZeroVelocities(velocities):
    # sometimes moveit gives us velocities of 0.0, when we want something
    # very slow, but the lib_robotis takes 0 as "unlimited"
    outVels = []
    for velocity in velocities:
        if velocity == 0.0:
            outVels.append(0.1)
        else:
            outVels.append(velocity)
    return outVels


class ybGripperServer:
    def __init__(self, name):
        self.fullname = name
        self.currentValue = 0.0
        gripperTopic = '/arm_1/gripper_controller/position_command'
        self.jppub = rospy.Publisher(gripperTopic, JointPositions)
        self.server = SimpleActionServer(self.fullname,
                                         GripperCommandAction,
                                         execute_cb=self.execute_cb,
                                         auto_start=False)
        self.server.start()

    def execute_cb(self, goal):
        rospy.loginfo(goal)
        self.currentValue = goal.command.position
        self.moveGripper(self.jppub, self.currentValue)
        attempts = 0
        # here we should be checking if the gripper has gotten to its goal
        for i in range(5):
            rospy.sleep(0.1)
            attempts += 1
        if attempts < 20:
            self.server.set_succeeded()
        else:
            self.server.set_aborted()

    def moveGripper(self, gPublisher, floatVal):
        jp = JointPositions()
        myPoison = Poison()
        myPoison.originator = 'arm1_gripper'
        myPoison.description = 'gripper command'
        myPoison.qos = 0.0
        jp.poisonStamp = myPoison
        nowTime = rospy.Time.now()
        jvl = JointValue()
        jvl.timeStamp = nowTime
        jvl.joint_uri = 'gripper_finger_joint_l'
        jvl.unit = 'm'
        jvl.value = floatVal
        jp.positions.append(jvl)
        jvr = JointValue()
        jvr.timeStamp = nowTime
        jvr.joint_uri = 'gripper_finger_joint_r'
        jvr.unit = 'm'
        jvr.value = floatVal
        jp.positions.append(jvr)
        gPublisher.publish(jp)

    def ybspin(self):
        rospy.sleep(0.1)


def ybserver():
    rospy.init_node('youbot_gripper_server')
    gripperCommandTopic = 'youbot_gripper/gripper_command'
    gripperServer = ybGripperServer(gripperCommandTopic)
    while not rospy.is_shutdown():
        gripperServer.ybspin()


if __name__ == '__main__':
    try:
        ybserver()
    except rospy.ROSInterruptException:
        pass
