#The ROS Node Subscriber template

#!/usr/bin/env python

#remove or add the library/libraries for ROS
import rospy, time, math, cv2, sys

#remove or add the message type
from geometry_msgs.msg import Quaternion

topic = 'obstacleDetection'
node_name = 'master'

#define function/functions to provide the required functionality
def name_callback(msg):
    rospy.loginfo("\nI heard:\n%s", msg)

if __name__=='__main__':
    #Add here the name of the ROS. In ROS, names are unique named.
    rospy.init_node(node_name)
    #subscribe to a topic using rospy.Subscriber class
    sub=rospy.Subscriber(topic, Quaternion, name_callback)
    rospy.spin()
