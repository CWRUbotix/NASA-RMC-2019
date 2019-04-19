#The ROS Node Subscriber template

#!/usr/bin/env python

#remove or add the library/libraries for ROS
import rospy
from depth_image_processing import CameraPosition

#remove or add the message type
from apriltags_ros.msg import Localization

topic = 'localization_data'
node_name = 'obstacleDetection'


def position_callback(msg):
    CameraPosition['x'] = msg.x
    CameraPosition['y'] = msg.y
    CameraPosition['azimuth'] = msg.theta
    print(msg.x, msg.y, msg.theta)

def update_position():
    #subscribe to a topic using rospy.Subscriber class
    sub=rospy.Subscriber(topic, Localization, position_callback)
    #rospy.spin()
