#!/usr/bin/env python

import Automodule.MotorCommand as mc

class DriveModule:
	def __init__(self, destination):
		self.destination = destination
		self.done = False
		self.currentPos = None
		self.obsFlag = False

	def done(self, robotPos):
		self.currentPos = robotPos
		if robotPos == self.destination:
			self.done = True
		return self.done

	def run(self):
		if self.obsFlag:
			raise ObstacleAlert()
		

	def setDone(self):
		self.done = True

	def stop(self):
		self.setDone()
		mc.drive_left_motor(0)
		mc.drive_right_motor(0)

class ObstacleAlert(Exception):
	def __init__(self):
		return
	def __str__(self):
		return 'Obstacle found exception not dealt with'

def driveTo(currentPos, pos):

	pass

# Given angle in radian the robot needs to turn and current angularvelocity
# Outputs tuple of left motor output and right motor output
# midAngle is when to start slowing down to avoid overshooting
# maxSpeed is output when not at risk of overshooting
# slowSpeed is output when finishing turn (at risk of overshooting)
def turn(angle, angularvelocity, midAngle, maxSpeed, slowSpeed):
	magAngle = angle
	if angle < 0:
		magAngle *= -1
	if magAngle > midAngle:
		if angle < 0:
			return -maxSpeed, maxSpeed
		else:
			return maxSpeed, -maxSpeed
	else:
		if angle < 0:
			return -slowSpeed, slowSpeed
		else:
			return slowSpeed, -slowSpeed


