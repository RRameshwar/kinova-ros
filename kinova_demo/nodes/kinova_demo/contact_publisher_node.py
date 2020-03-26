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

def getKey():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


if __name__ == '__main__':
    settings = termios.tcgetattr(sys.stdin)
    pub_userContact = rospy.Publisher('user_contact', Int16, queue_size=10)
    pub_trueContact = rospy.Publisher('true_contact', Int16, queue_size=10)
    pub_changeState = rospy.Publisher('change_state', Int16, queue_size=10)

    rospy.init_node('contact_publisher')
    rate = rospy.Rate(60)
    
    while not rospy.is_shutdown():
	    key = getKey()        
	    if key == 's':
	        user_contact_msg = Int16(data = 100)
	        print("Sent contact!")
	        pub_userContact.publish(user_contact_msg)
	    if key == 'l':
	        true_contact_msg = Int16(data = 100)
	        print("Sent contact!")
	        pub_trueContact.publish(true_contact_msg)
	    if key == 'c':
	    	pub_changeState.publish(Int16(data = 1))
	    if key == 'x':
	        rospy.signal_shutdown("User Ended Program") 

	    rate.sleep()
