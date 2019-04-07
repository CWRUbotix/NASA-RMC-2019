#!/usr/bin/env python

import rospy
from hci.msg import motorCommand

"""
Functions for controlling motors
"""

def drive_left_motor(pub, value):
    pub.publish(motorID = 0, value = value)

def drive_right_motor(pub, value):
    pub.publish(motorID = 1, value = value)

def deposition_motor(pub, value):
    pub.publish(motorID = 2, value = value)

def bucket_drive_motor(pub, value):
    pub.publish(motorID = 3, value = value)

def bucket_translation_motor(pub, value):
    pub.publish(motorID = 4, value = value)

def bucket_angle_actuator(pub, value):
    pub.publish(motorID = 5, value = value)

