import rospy
from geometry_msgs.msg import Point
from obstacle_detection.msg import Path

topic = 'path_data'


def update_points(points):
	def position_callback(msg):
		print('Message callback', msg.points)
		points = msg.points
	return position_callback

def get_path(points):
    sub=rospy.Subscriber(topic, Path, update_points(points))