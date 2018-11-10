#!/usr/bin/env python
import rospy
from obstacle_detection.msg import Obstacle

topic = 'obstacleDetection'
node_name = 'master'

def send_obstacle_data(obs):
    global topic
    global node_name
    try:
        pub = rospy.Publisher(topic, Obstacle, queue_size=10)
	msg = Obstacle()
	msg.x = obs.x
	msg.y = obs.y
	msg.z = obs.z
	msg.diameter = obs.diameter
        rospy.init_node(node_name, anonymous=True)
        rospy.loginfo(msg)
        pub.publish(msg)
    except rospy.ROSInterruptException as e:
	print(e.getMessage())
        pass
