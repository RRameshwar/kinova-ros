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

prefix = 'j2n6s300_'


#BOX TOP GRASP
pos1_prep = [0.0359343886375, -0.355574041605, 0.221754789352, 0.177286446095, 0.762554585934, -0.580067753792, -0.224947333336]
pos1 = [0.0359343886375, -0.355574041605, 0.221754789352, 0.177286446095, 0.762554585934, -0.580067753792, -0.224947333336] # GOOD
pos2 = [0.0522202402353, -0.405868023634, 0.223289772868, -0.0220888052136, -0.807200670242, 0.589077472687, 0.0304453372955] # BAD
pos3 = [0.0362568795681, -0.358544528484, 0.225575596094, 0.252614378929, 0.746703505516, -0.603892624378, -0.11804100126] #ALSO BAD


#BOX SIDE GRASP
pos4 = [0.146627187729, -0.425589442253, 0.161293938756, 0.56351518631, -0.61013007164, 0.369820535183, 0.416442811489] # GOOD
pos5 = [0.146738886833, -0.425929933786, 0.156194254756, 0.454542994499, -0.697697639465, 0.431119531393, 0.347483098507] # MEDIOCRE/BAD
pos6 = [0.145677149296, -0.424433946609, 0.181024760008, 0.50988972187, -0.659492135048, 0.399094760418, 0.381845504045] #GOOD

poses = [pos1, pos2, pos3, pos4, pos5]

#Fingers half closed: [400, 70, 100, 100, 100] 

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

def sendCommand(goal, isquat):
    pos = goal[0:3]
    if isquat == True:
        ori = goal[3:7]
    else:
        ori = EulerXYZ2Quaternion(goal[3:7])
    
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
    rospy.init_node(prefix + 'jaco_positions')
    n = int(sys.argv[1])
    
    pos = poses[n-1]
    
    sendCommand(pos, 1)
