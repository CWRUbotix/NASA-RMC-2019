from collections import deque

import rospy
from PathPlanning.PathPlanning import Position #import specific functions later

"""
Main Module for Autonomy operation
"""


""" HARD CODED TABLES (dictionary)"""
interruptible = [4, 8, 11, 14] #Task IDs whose time can be modified

tasks = #Map from ID to time allocated.
{ 
	0: -1
	1: 20
	2: 20
	3: 50
	4: 100
	5: 20
	6: 50
	7: 25
	8: 15
	9: 20
	10: 50
	11: 100
	12: 20
	13: 50
	14: 25
	15: 15
	16: 0
}

#return true if robot arrived in a position within the the time false if not.
def goTo(pos, ID):
	# First you request for the path

	# Given ID start the timer with assigned time

	# Start loop of driveTo method calls with given path



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

#Give start positio nand end position to PathPlanning module to get path
#This function returns path
def requestPath(start, end):
	pass

#Start DriveControl subprocess with destination position.
#Return an DriveControl object that you can call done() function that indicates when the robot is at the position
def driveTo(pos):
	pass

#Alerts Path Planning submodule with new obstacle found and get a new path
def foundObstacle(currentPos, obs):
	pass

def main():
	rospy.init_node("automodule")
	rospy.on_shutdown(shutdownRoutine)

	# Initialize RoutineMaker threads here

	rospy.spin()

if __name__ == "__main__": main()
