#!/usr/bin/env python
import rospy
from obstacle_detection.msg import Obstacle

topic = 'obstacleDetection'
node_name = 'obstacle_detection_node'


def send_obstacle_data(obs):
    """
    Publishes the ID, XYZ coordinates, and diameter in a custom Obstacle message.  All measurements are in meters

    Args:
        obs (:obj:`Obstacle`) : obstacle object containing the ID, coordinates, and diameter of the obstacle to publish

    """
    global topic
    global node_name
    try:
        pub = rospy.Publisher(topic, Obstacle, queue_size=10)
        msg = Obstacle()
        msg.obsID = obs.id
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
