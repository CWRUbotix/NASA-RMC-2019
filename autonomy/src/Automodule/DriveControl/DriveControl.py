
import rospy

from PathPlanning.PathPlanning import Position

class DriveModule:
	def __init__(self, destination):
		self.destination = destination
		self.done = False

	def done(self):
		return self.done

	def run(self):
		try:
			rospy.init_node('Drive Control ' + str(self.destination))
			rospy.on_shutdown(self.setDone)
		except ROSInitException as e:
			raise e

		
		
		rospy.spin()

	def setDone(self):
		self.done = True

	def stop(self):


def driveTo(pos):
	
