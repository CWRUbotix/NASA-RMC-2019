#!/usr/bin/env python
import rospy
from obstacle_detection.msg import Obstacle
from std_msgs.msg import Float32

obstacle_topic = 'obstacleDetection'
optical_flow_topic = 'opticalFlow'
node_name = 'obstacle_detection_node'


def setup_obstacle_node():
    global node_name
    rospy.init_node(node_name, anonymous=True)


def send_obstacle_data(obs):
    """
    Publishes the ID, XYZ coordinates, and diameter in a custom Obstacle message.  All measurements are in meters

    Args:
        obs (:obj:`Obstacle`) : obstacle object containing the ID, coordinates, and diameter of the obstacle to publish

    """
    global obstacle_topic
    try:
        pub = rospy.Publisher(obstacle_topic, Obstacle, queue_size=10)
        msg = Obstacle()
        msg.obsID = obs.id
        msg.x = obs.x
        msg.y = obs.y
        msg.z = obs.z
        msg.diameter = obs.diameter
        #rospy.loginfo(msg)
        pub.publish(msg)
    except rospy.ROSInterruptException as e:
        print(e.getMessage())
        pass


def send_optical_flow_data(y):
    global optical_flow_topic
    try:
        pub = rospy.Publisher(optical_flow_topic, Float32, queue_size=10)
        msg = Float32()
        msg.data = y
        print('Y Displacement: %.3f' % y)
        #rospy.loginfo(msg)
        pub.publish(msg)
    except rospy.ROSInterruptException as e:
        print(e.getMessage())
        pass
