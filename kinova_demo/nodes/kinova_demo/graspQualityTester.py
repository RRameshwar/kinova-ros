#! /usr/bin/env python
"""A helper program to test cartesian goals for the JACO and MICO arms."""

import roslib; roslib.load_manifest('kinova_demo')
import rospy

import sys
import numpy as np
import random

import actionlib
import kinova_msgs.msg
import std_msgs.msg
import geometry_msgs.msg
from std_msgs.msg import Int16
from std_msgs.msg import Int16MultiArray


import math
import argparse

import csv


import sys, select, termios, tty

prefix = 'j2n6s300_'


#BOX TOP GRASP
pos1_prep = [0.0359343886375, -0.355574041605, 0.221754789352, 0.177286446095, 0.762554585934, -0.580067753792, -0.224947333336]
pos1 = [1, 0.0359343886375, -0.355574041605, 0.221754789352, 0.177286446095, 0.762554585934, -0.580067753792, -0.224947333336] # GOOD
pos2 = [2, 0.0522202402353, -0.405868023634, 0.223289772868, -0.0220888052136, -0.807200670242, 0.589077472687, 0.0304453372955] # BAD
pos3 = [3, 0.0362568795681, -0.358544528484, 0.225575596094, 0.252614378929, 0.746703505516, -0.603892624378, -0.11804100126] #ALSO BAD


#BOX SIDE GRASP
pos4 = [4, 0.146627187729, -0.425589442253, 0.161293938756, 0.56351518631, -0.61013007164, 0.369820535183, 0.416442811489] # GOOD
pos5 = [5, 0.146738886833, -0.425929933786, 0.156194254756, 0.454542994499, -0.697697639465, 0.431119531393, 0.347483098507] # MEDIOCRE/BAD
pos6 = [6, 0.145677149296, -0.424433946609, 0.181024760008, 0.50988972187, -0.659492135048, 0.399094760418, 0.381845504045] #GOOD

poses = [pos1, pos2, pos3, pos4, pos5, pos6]
poses_ordered = []

guesses = []
actual = []

global counter
counter = 0

global made_contact
made_contact = 0



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

def sendCommand(goal):
    pos = goal[1:4]
    ori = goal[4:8]  
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

def calculateScore():
    print(guesses)
    print(actual)

    bad_r = 0
    good_r = 0
    med_r = 0

    bad_w = 0
    good_w = 0
    med_w = 0

    for i in range(0, len(guesses)):
        guess = guesses[i]
        truth = actual[i]
        
        if truth == 'B':
            if guess == 'B':
                bad_r += 1
            else:
                bad_w += 1
        if truth == 'G':
            if guess == 'G':
                good_r += 1
            else:
                good_w += 1
        if truth == 'M':
            if guess == 'M':
                med_r += 1
            else:
                med_w += 1

    total_right = bad_r + good_r + med_r
    total_wrong = bad_w + good_w + med_w

    print("Score:")
    print(["Total Correct: ", total_right])
    print(["Total Wrong: ", total_wrong])
    print(["Bad Right, Bad Wrong", bad_r, bad_w])
    print(["Good Right, Good Wrong", good_r, good_w])
    print(["Med Right, Med Wrong", med_r, med_w])


def user_callback(msg):
    global contact_guess
    print("got contact!")
    pub_placed.publish(Int16(data=800))

def state_callback(msg):
    global counter
    print("counter up!")
    counter += 1


if __name__ == '__main__':
    global counter
    global made_contact
    settings = termios.tcgetattr(sys.stdin)
    jump = 0.025

    test = 1


    pub = rospy.Publisher('fingersPos', Int16MultiArray, queue_size=10)
    pub_placed = rospy.Publisher('objectPlaced', Int16, queue_size=10)
    rospy.Subscriber("user_contact", Int16, user_callback)
    rospy.Subscriber("change_state", Int16, state_callback)
    
    rospy.init_node(prefix + 'jaco_positions')
    rate = rospy.Rate(10)
    user_test = poses[:]
    user_test.extend(poses)
    user_test.extend(poses)
    random.shuffle(user_test)

    for i in user_test:
        poses_ordered.append(i[0])

    print(poses_ordered)

    closed_pos = [0, 0, 0, 0, 0]
    closed_thumb = [0, 2000, 2000, 2000, 2000]
    open_pos = [2000, 2000, 2000, 2000, 2000]


    ## OPEN/CLOSE THE HAND
    # while not rospy.is_shutdown():
    # s = raw_input("Press enter to grasp the object")
    # msg = Int16MultiArray(data=closed_pos)
    # pub.publish(msg)

    # s = raw_input("Press enter to release the object")
    # msg = Int16MultiArray(data=open_pos)
    # pub.publish(msg)

    # rate.sleep()
    

    ## TEST TAPPING OR PERTURBING
    #sendCommand(pos1_prep)

    while not rospy.is_shutdown():
        pub_placed.publish(Int16(data=made_contact))
        #if counter == 0:
            #sendCommand(pos4)
        # if counter == 1:
        #     print("pos1")
        #     sendCommand(pos4)
        
        if counter == 1:
            msg = Int16MultiArray(data=closed_thumb)
            pub.publish(msg)

        if counter == 2:
            msg = Int16MultiArray(data=closed_pos)
            pub.publish(msg)
        
        elif counter == 2:
            lifted_pos = pos4[:]
            lifted_pos[3] += 0.1
            print(lifted_pos)
            #sendCommand(lifted_pos)
        
        # elif counter == 4:
        #     lowered_pos = pos4[:]
        #     lowered_pos[2] -= 0.01
        #     print(lowered_pos)
        #     sendCommand(lowered_pos)
        
        # elif counter == 5:
        #     sendCommand(pos4)

        elif counter == 3:
            msg = Int16MultiArray(data=open_pos)
            pub.publish(msg)

        rate.sleep()


    ## GRASP QUALITY USER TEST
    # while test == 1:
    #     # s = raw_input("Press enter to move to position" + str(i+1))
    #     # sendCommand(user_test[i])
    #     s = raw_input("Press enter to grasp the object")
    #     msg = Int16MultiArray(data=closed_pos)
    #     pub.publish(msg)

    #     s = raw_input("What does the user guess?")
    #     guesses.append(s)

    #     # s = raw_input("Press enter to lift the object")
    #     # lifted_pos = user_test[i][:]
    #     # lifted_pos[3] += 0.06
    #     # print(lifted_pos)
    #     # sendCommand(lifted_pos)

    #     s = raw_input("What is the actual grasp quality?")
    #     actual.append(s)
        
    #     s = raw_input("Press enter to release the object")
    #     msg = Int16MultiArray(data=open_pos)
    #     pub.publish(msg)

    #     s = raw_input("Do you want to continue?")
        
    #     if s == 'N' or s == 'n':
    #         test = 0

    #     else:
    #         test = 1

    # calculateScore()



