#The ROS Node Subscriber template

#!/usr/bin/env python

#remove or add the library/libraries for ROS
import rospy
import math
from depth_image_processing import CameraPosition

#remove or add the message type
from apriltags_ros.msg import Localization

topic = 'localization_data'
node_name = 'master'


def position_callback(msg):
    CameraPosition['x'] = msg.x
    CameraPosition['y'] = msg.y
    CameraPosition['azimuth'] = msg.theta * 180 / math.pi
    print('Localization:', msg.x, msg.y, msg.theta)

def update_position():
    #Add here the name of the ROS. In ROS, names are unique named.
    #subscribe to a topic using rospy.Subscriber class
    sub=rospy.Subscriber(topic, Localization, position_callback)
