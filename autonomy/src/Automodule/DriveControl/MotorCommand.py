#!/usr/bin/env python

import rospy
from hci.msg import motorCommand

"""
Functions for controlling motors
"""

def drive_left_motor(value):
    pub = rospy.Publisher('motorCommand', motorCommand, queue_size = 10)
    pub.publish(motorID = 0, value = value)

def drive_right_motor(value):
    pub = rospy.Publisher('motorCommand', motorCommand, queue_size = 10)
    pub.publish(motorID = 1, value = value)

def deposition_motor(value):
    pub = rospy.Publisher('motorCommand', motorCommand, queue_size = 10)
    pub.publish(motorID = 2, value = value)

def bucket_drive_motor(value):
    pub = rospy.Publisher('motorCommand', motorCommand, queue_size = 10)
    pub.publish(motorID = 3, value = value)

def bucket_translation_motor(value):
    pub = rospy.Publisher('motorCommand', motorCommand, queue_size = 10)
    pub.publish(motorID = 4, value = value)

def bucket_angle_actuator(value):
    pub = rospy.Publisher('motorCommand', motorCommand, queue_size = 10)
    pub.publish(motorID = 5, value = value)

