#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from PathPlanning.PathPlanning import Position #import specific functions later

"""
Main Module for Autonomy operation
"""


""" HARD CODED TABLES (dictionary)"""
interruptible = [4, 8, 11, 14] #Task IDs whose time can be modified
robotPos = None
destination = None
drive = None
currentID = 0
#Map from ID to time allocated.
tasks = {
	0: -1,
	1: 20,
	2: 20,
	3: 50,
	4: 100,
	5: 20,
	6: 50,
	7: 25,
	8: 15,
	9: 20,
	10: 50,
	11: 100,
	12: 20,
	13: 50,
	14: 25,
	15: 15,
	16: 0
}

#return true if robot arrived in a position within the the time false if not.
def goTo(dest, ID):
	global drive, destination, currentID
	destination = dest
	# First you request for the path
	path = requestPath(robotPos, destination)

	# Given ID start the timer with assigned time
	tGoal = rospy.get_time() + tasks[ID]

	# Start loop of driveTo method calls with given path
	for pos in path:
		drive = driveTo(pos)
		while not drive.done(robotPos):
			rospy.sleep(.1)

	drive = None
	destination = None
	currentID += 1

	if rospy.get_time() > tGoal:
		return False
	else:
		return True

def updatePos(pos):
	global robotPos
	robotPos = pos

#Alerts Path Planning submodule with new obstacle found and get a new path
def foundObstacle(obs):
	if drive is not None:
		drive.stop()
		goTo(destination, currentID)

"""Sensor Subscription"""
def subscribe():
	#todo Fill in names
	rospy.Subscriber('', String, updatePos)
	rospy.Subscriber('', String, foundObstacle)


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

#Give start position and end position to PathPlanning module to get path
#This function returns path
def requestPath(start, end):
	pass

#Start DriveControl subprocess with destination position.
#Return an DriveControl object that you can call done() function that indicates when the robot is at the position
def driveTo(pos):
	pass

def main():
	rospy.init_node("automodule")
	rospy.on_shutdown(shutdownRoutine)
	subscribe()

	# Initialize RoutineMaker threads here

	rospy.spin()

if __name__ == "__main__": main()
