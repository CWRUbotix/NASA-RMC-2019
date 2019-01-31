from collections import deque

import rospy


"""
Main Module for Autonomy operation
"""


""" HARD CODED TABLES (dictionary)"""
interruptible = [4, 8, 11, 14] #Task IDs whose time can be modified

tasks = #Map from ID to time allocated.
{}


"""Routine Builder Stuff"""

"""
Takes in task ID and create list of function calls that
"""
def createRoutine(ID):
	pass

def modifyTimes(timeLeft):
	pass

def shutdownRoutine():
	pass


def main():
	rospy.init_node("automodule")
	rospy.on_shutdown(shutdownRoutine)

	# Initialize RoutineMaker threads here

	rospy.spin()
	

if __name__ == "__main__": main()
