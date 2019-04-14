#The ROS Node Subscriber template

#!/usr/bin/env python

#remove or add the library/libraries for ROS
import rospy
from from depth_image_processing import CameraPosition

#remove or add the message type
from geometry_msgs.msg import Pose2D

topic = 'localization_data'
node_name = 'master'


def position_callback(msg):
    CameraPosition['x'] = msg.x
    CameraPosition['y'] = msg.y
    CameraPosition['azimuth'] = msg.theta
    print(msg.x, msg.y, msg.theta)

def update_position():
    #Add here the name of the ROS. In ROS, names are unique named.
    rospy.init_node(node_name)
    #subscribe to a topic using rospy.Subscriber class
    sub=rospy.Subscriber(topic, Pose2D, position_callback)
