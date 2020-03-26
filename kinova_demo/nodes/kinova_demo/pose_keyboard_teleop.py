#! /usr/bin/env python
"""A helper program to test cartesian goals for the JACO and MICO arms."""

import roslib; roslib.load_manifest('kinova_demo')
import rospy

import sys
import numpy as np

import actionlib
import kinova_msgs.msg
import std_msgs.msg
import geometry_msgs.msg


import math
import argparse

import csv


import sys, select, termios, tty

""" Global variable """
arm_joint_number = 0
finger_number = 0
prefix = 'j2n6s300_'
finger_maxDist = 18.9/2/1000  # max distance for one finger
finger_maxTurn = 6800  # max thread rotation for one finger
currentCartesianCommand = [0.212322831154, -0.257197618484, 0.509646713734, 1.63771402836, 1.11316478252, 0.134094119072] # default home in unit mq
startPos = [0.31232283115400006, -0.8071976184840001, 0.2596467137339997, 0.6429107629876011, 0.3184928002651115, 0.42383156804281624, 0.5528063756364934]
savedCartesianCommand = []

commandtype = 'POSE'

grasp = 2000

action = False

bindings = {
    'w':1,
    's':-1,
    'a':2,
    'd':-2,
    'q':3,
    'e':-3, 

    'u':4,
    'o':-4,
    'i':5,
    'k':-5,
    'j':6,
    'l':-6,
     }


def cartesian_pose_client(position, orientation):
    """Send a cartesian goal to the action server."""
    action_address = '/' + prefix + 'driver/pose_action/tool_pose'
    client = actionlib.SimpleActionClient(action_address, kinova_msgs.msg.ArmPoseAction)
    client.wait_for_server()

    goal = kinova_msgs.msg.ArmPoseGoal()
    goal.pose.header = std_msgs.msg.Header(frame_id=(prefix + 'link_base'))
    goal.pose.pose.position = geometry_msgs.msg.Point(
        x=position[0], y=position[1], z=position[2])
    goal.pose.pose.orientation = geometry_msgs.msg.Quaternion(
        x=orientation[0], y=orientation[1], z=orientation[2], w=orientation[3])

    # print('goal.pose in client 1: {}'.format(goal.pose.pose)) # debug

    client.send_goal(goal)
    action = True
    if client.wait_for_result(rospy.Duration(10.0)):
        print('success!')
        action = False
        return client.get_result()
    else:
        client.cancel_all_goals()
        action = False
        print('the cartesian action timed-out')
        return None


def getKey():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def sendCommand(quat):
    pos = currentCartesianCommand[0:3]
    if quat == 1:
        ori = currentCartesianCommand[3:7]
    else:
        ori = EulerXYZ2Quaternion(currentCartesianCommand[3:7])
    print(pos, "  ", ori)
    cartesian_pose_client(pos, ori)

def EulerXYZ2Quaternion(EulerXYZ_):
    tx_, ty_, tz_ = EulerXYZ_[0:3]
    sx = math.sin(0.5 * tx_)
    cx = math.cos(0.5 * tx_)
    sy = math.sin(0.5 * ty_)
    cy = math.cos(0.5 * ty_)
    sz = math.sin(0.5 * tz_)
    cz = math.cos(0.5 * tz_)

    qx_ = sx * cy * cz + cx * sy * sz
    qy_ = -sx * cy * sz + cx * sy * cz
    qz_ = sx * sy * cz + cx * cy * sz
    qw_ = -sx * sy * sz + cx * cy * cz

    Q_ = [qx_, qy_, qz_, qw_]
    return Q_

if __name__ == '__main__':
    settings = termios.tcgetattr(sys.stdin)
    jump = 0.025
    rospy.init_node(prefix + 'keyboard_teleop')

    while not rospy.is_shutdown():
        key = getKey()
        if key in bindings:
            currentCartesianCommand[abs(bindings[key])-1] += jump*np.sign(bindings[key])
            sendCommand(0)
        elif key == 'h':
            currentCartesianCommand = startPos
            sendCommand(1)
        elif key == 'g':
            if grasp == 2000:
                grasp = 1000
            else:
                grasp = 2000
        elif key == 't': #test a lift
            savedCartesianCommand = currentCartesianCommand
            currentCartesianCommand[2] += 0.75
            sendCommand(0)
            while action == True:
                pass
            currentCartesianCommand[2] -= 0.75
            sendCommand(0)
            while action == True:
                pass
        elif key == 'b':
            if jump == 0.025:
                jump = 0.5
            else:
                jump = 0.025
            print("Jump is now", jump)
        elif key == 'y':
            name = raw_input()
            with open("positions.csv") as testCSV:
                writeCSV = csv.write(testCSV, delimiter=',')
                writeCSV.writerow([name, savedCartesianCommand])
        else:
            if key == 'x':
                rospy.signal_shutdown("User Ended Program") 