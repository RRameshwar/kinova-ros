#! /usr/bin/env python


import roslib
roslib.load_manifest('kinova_demo')
import rospy

import sys

import math

import actionlib
import kinova_msgs.msg

import argparse

from std_msgs.msg import Float32MultiArray


""" Global variable """
arm_joint_number = 0
finger_number = 0
prefix = 'j2n6s300'
finger_maxDist = 18.9/2/1000  # max distance for one finger
finger_maxTurn = 6800  # max thread rotation for one finger
currentJointCommand = [0, 0, 0, 0, 0, 0] # number of joints is defined in __main__
currentFingerPosition = [0.0, 0.0, 0.0]


def joint_angle_client(angle_set):
    """Send a joint angle goal to the action server."""
    action_address = '/' + prefix + 'driver/joints_action/joint_angles'
    client = actionlib.SimpleActionClient(action_address,
                                          kinova_msgs.msg.ArmJointAnglesAction)
    client.wait_for_server()

    goal = kinova_msgs.msg.ArmJointAnglesGoal()

    goal.angles.joint1 = angle_set[0]
    goal.angles.joint2 = angle_set[1]
    goal.angles.joint3 = angle_set[2]
    goal.angles.joint4 = angle_set[3]
    goal.angles.joint5 = angle_set[4]
    goal.angles.joint6 = angle_set[5]
    goal.angles.joint7 = angle_set[6]

    client.send_goal(goal)
    if client.wait_for_result(rospy.Duration(20.0)):
        return client.get_result()
    else:
        print('        the joint angle action timed-out')
        client.cancel_all_goals()
        return None

def gripper_client(finger_positions):
    """Send a gripper goal to the action server."""
    action_address = '/' + prefix + 'driver/fingers_action/finger_positions'

    client = actionlib.SimpleActionClient(action_address,
                                          kinova_msgs.msg.SetFingersPositionAction)
    client.wait_for_server()

    goal = kinova_msgs.msg.SetFingersPositionGoal()
    goal.fingers.finger1 = float(finger_positions[0])
    goal.fingers.finger2 = float(finger_positions[1])
    # The MICO arm has only two fingers, but the same action definition is used
    if len(finger_positions) < 3:
        goal.fingers.finger3 = 0.0
    else:
        goal.fingers.finger3 = float(finger_positions[2])
    client.send_goal(goal)
    if client.wait_for_result(rospy.Duration(5.0)):
        return client.get_result()
    else:
        client.cancel_all_goals()
        rospy.WARN('        the gripper action timed-out')
        return None

def callback(data):
    arm_angles = data[0:7]
    gripper_angles = data[7:10]
    
    gripper_client(gripper_angles)
    joint_angle_client(arm_angles)



if __name__ == '__main__':
    rospy.init_node(prefix + 'gripper_workout')
    rospy.Subscriber("kinova_glove_control", Float32MultiArray, callback)
    rospy.spin()
