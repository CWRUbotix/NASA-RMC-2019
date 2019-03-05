#!/usr/bin/env python

import Automodule.MotorCommand as mc

class DriveModule:
	def __init__(self, destination):
		self.destination = destination
		self.done = False

	def done(self, robotPos):
		if robotPos == self.destination:
			self.done = True
		return self.done

	def run(self):
		pass
		

	def setDone(self):
		self.done = True

	def stop(self):
		self.setDone()
		mc.drive_left_motor(0)
		mc.drive_right_motor(0)

def driveTo(pos):
	pass

# Given angle in radian the robot needs to turn and current angularvelocity
# Outputs tuple of left motor output and right motor output
def turn(angle, angularvelocity):
	pass

