#!/usr/bin/env python

import rospy
import math
from std_msgs.msg import String
from PathPlanning.PathPlanning import Position #import specific functions later
from DriveControl.DriveControl import DriveModule, ObstacleAlert
import MotorCommand as mc

""" 
Main Module for Autonomy operation
"""


""" HARD CODED TABLES (dictionary)"""
#The alignment location
COLLECTION_BIN = Position(1.23, 0.825, math.pi)
interruptible = [4, 8, 11, 14] #Task IDs whose time can be modified
robotPos = None
destination = None
drive = None
currentID = 0
obstacles = []
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
def goTo(dest):
	global drive, destination, currentID
	destination = dest
	# First you request for the path
	path = requestPath(robotPos, destination)

	# Given ID start the timer with assigned time
	tGoal = rospy.get_time() + tasks[currentID]

	# Start loop of driveTo method calls with given path
	try:
		for pos in path:
			drive = DriveModule(pos)
			while not drive.done(robotPos):
				motorvalues = drive.run()
				mc.drive_left_motor(motorvalues[0])
				mc.drive_right_motor(motorvalues[1])
				rospy.sleep(.1)
	except ObstacleAlert:
		return False

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
	obstacles.append(obs)
	if drive is not None:
		drive.stop()
		goTo(destination)

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

"""
Functions that belong to specific tasks
naming: task_<ID>
Should not be taking in inputs. if you need something, make it global
"""

def task_1():
	pass

#Cycle 1 digging goal : 100
def task_4():
	driveStrength = 40
	translationStrength = 10
	angleStrength = 10
	#Is there a way to know if it's fully angled down?
	mc.bucket_angle_actuator(angleStrength)
	rospy.sleep(15)
	mc.bucket_angle_actuator(0)
	mc.bucket_drive_motor(driveStrength)
	mc.bucket_translation_motor(translationStrength)
	rospy.sleep(60)
	mc.bucket_drive_motor(0)
	mc.bucket_translation_motor(-1 * translationStrength)
	rospy.sleep(10)
	mc.bucket_translation_motor(0)
	mc.bucket_angle_actuator(-angleStrength)
	rospy.sleep(15)
	mc.bucket_angle_actuator(0)

#Dumping (Cycle1): 15
def task_8():
	#Need to check the actual angle depending on the incline of the bucket
	angleStrength = 225;
	mc.bucket_angle_actuator(angleStrength)
	rospy.sleep(7)
	mc.bucket_angle_actuator(-angleStrength)
	rospy.sleep(7)

#Cycle 2 digging goal : 100
def task_11():
	driveStrength = 40
	translationStrength = 10
	angleStrength = 10
	# Is there a way to know if it's fully angled down?
	mc.bucket_angle_actuator(angleStrength)
	rospy.sleep(15)
	mc.bucket_angle_actuator(0)
	mc.bucket_drive_motor(driveStrength)
	mc.bucket_translation_motor(translationStrength)
	rospy.sleep(60)
	mc.bucket_drive_motor(0)
	mc.bucket_translation_motor(-1 * translationStrength)
	rospy.sleep(10)
	mc.bucket_translation_motor(0)
	mc.bucket_angle_actuator(-angleStrength)
	rospy.sleep(15)
	mc.bucket_angle_actuator(0)

#Dumping (Cycle2): 15
def task_15():
	angleStrength = 225
	mc.bucket_angle_actuator(angleStrength)
	rospy.sleep(7)
	mc.bucket_angle_actuator(-angleStrength)
	rospy.sleep(7)

"""
functions for tasks END
"""

def modifyTimes(timeLeft):
	pass

def shutdownRoutine():
	pass

def main():
	rospy.init_node("automodule")
	rospy.on_shutdown(shutdownRoutine)
	subscribe()

	# Initialize RoutineMaker threads here

	rospy.spin()

if __name__ == "__main__": main()
