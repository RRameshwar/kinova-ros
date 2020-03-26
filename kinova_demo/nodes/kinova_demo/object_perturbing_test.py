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
from std_msgs.msg import Int16


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

pos1 = [0.0359343886375, -0.355574041605, 0.221754789352, 0.177286446095, 0.762554585934, -0.580067753792, -0.224947333336] # GOOD
pos2 = [0.146627187729, -0.425589442253, 0.161293938756, 0.56351518631, -0.61013007164, 0.369820535183, 0.416442811489] # GOOD
pos3 = [0.145677149296, -0.424433946609, 0.181024760008, 0.50988972187, -0.659492135048, 0.399094760418, 0.381845504045] #GOOD

poses = [pos1, pos2, pos3]


commandtype = 'POSE'

grasp = 2000

action = False

global contact_guess
contact_guess = 0

global contact_true
contact_true = 0

def user_callback(msg):
    global contact_guess
    print("got contact!")
    contact_guess = 100

def true_callback(msg):
    global contact_true
    print("got contact true!")
    contact_true = 100

if __name__ == '__main__':
    global contact_guess
    global contact_true

    settings = termios.tcgetattr(sys.stdin)
    jump = 0.025
    
    pub_userContact = rospy.Publisher('user_contact_data', Int16, queue_size=10)
    pub_trueContact = rospy.Publisher('true_contact_data', Int16, queue_size=10)
    
    rospy.Subscriber("true_contact", Int16, true_callback)
    rospy.Subscriber("user_contact", Int16, user_callback)

    rospy.init_node(prefix + 'object_perturb_data_collect')
    rate = rospy.Rate(10)
    
    while not rospy.is_shutdown():
        user_contact_msg = Int16(data=contact_guess)
        true_contact_msg = Int16(data=contact_true)
        
        pub_userContact.publish(user_contact_msg)
        pub_trueContact.publish(true_contact_msg)
        
        contact_guess = 0
        contact_true = 0
        
        rate.sleep()