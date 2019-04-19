#!/usr/bin/env python
#Janky simulator

import rospy
from geometry_msgs.msg import Pose2D
from hci.msg import motorCommand

wait = True
locPub = rospy.Publisher('localization_data', Pose2D, queue_size=10)

def sendFalseLocData(x, y, theta):
    global locPub
    locPub.publish(x=x, y=y, theta=theta)
    print 'sent'

def confirmMotorMsg(msg):
    print 'got motor command'
    global wait
    wait = False

def main():
    global wait
    rospy.init_node('simulator')
    raw_input('simulator ready. press enter to start')
    increment = 0.1
    x = 0
    sendFalseLocData(0, 0, 0)
    rospy.Subscriber('motorCommand', motorCommand, confirmMotorMsg)
    while x < 1.5:
        print 'wating motorcommand'
        while wait:
            rospy.sleep(0.01)
        print 'motorcommand received'
        x += increment
        sendFalseLocData(x, 0, 0)
        wait = True
    rospy.spin()

if __name__ == '__main__' :
    main()
