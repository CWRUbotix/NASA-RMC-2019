#!/usr/bin/env python

import rospy
import sys
import math
from Automodule import shutdownRoutine
import PathPlanning.PathPlanning as pp
from geometry_msgs.msg import Pose2D
from hci.msg import sensorValue
from obstacle_detection.msg import Obstacle


class Robot_state:
    def __init__(self):
        self.currentPos = None
        self.gyro_0x = 0.0
        self.gyro_0y = 0.0
        self.gyro_0z = 0.0
        self.gyro_1x = 0.0
        self.gyro_1y = 0.0
        self.gyro_1z = 0.0
        self.port_drive_rpm = 0.0
        self.starboard_drive_rpm = 0.0
        self.dep_load_cell = 0.0
        self.dep_lower_limit_switch = False
        self.dep_upper_limit_switch = False
        self.bcarm_lower_limit_switch = False # When translated all the way up
        self.bcarm_upper_limit_switch = False # When translated all the way down
        #self.bc_fore_limit_switch = False
        #self.bc_alt_limit_switch = False
        self.obstacles = []

    def getCurrentPos(self):
        return self.currentPos

    def setCurrentPos(self, currentPos):
        self.currentPos = currentPos

    def getGyroX(self):
        return (self.gyro_0x + self.gyro_1x) / 2.0

    def setGyro0X(self, gyro_x):
        self.gyro_0x = gyro_x

    def setGyro1X(self, gyro_x):
        self.gyro_1x = gyro_x

    def getGyroY(self):
        return (self.gyro_0y + self.gyro_1y) / 2.0

    def setGyro0Y(self, gyro_y):
        self.gyro_0y = gyro_y

    def setGyro1Y(self, gyro_y):
        self.gyro_1y = gyro_y

    def getGyroZ(self):
        return (self.gyro_0z + self.gyro_1z) /2

    def setGyro0Z(self, gyro_z):
        self.gyro_0z = gyro_z

    def setGyro1Z(self, gyro_z):
        self.gyro_1z = gyro_z

    def getPortRPM(self):
        return self.port_drive_rpm

    def setPortRPM(self, port_drive_rpm):
        self.port_drive_rpm = port_drive_rpm

    def getStarRPM(self):
        return self.starboard_drive_rpm

    def setStarRPM(self, starboard_drive_rpm):
        self.starboard_drive_rpm = starboard_drive_rpm

    def getDepLoad(self):
        return self.dep_load_cell

    def setDepLoad(self, dep_load_cell):
        self.dep_load_cell = dep_load_cell

    def getDepLowerLimit(self):
        return self.dep_lower_limit_switch

    def setDepLowerLimit(self, dep_lower_limit_switch):
        self.dep_lower_limit_switch = dep_lower_limit_switch

    def getDepUpperLimit(self):
        return self.dep_upper_limit_switch

    def setDepUpperLimit(self, dep_upper_limit_switch):
        self.dep_upper_limit_switch = dep_upper_limit_switch

    def getBCArmLowerLimit(self):
        return self.bcarm_lower_limit_switch

    def setBCArmLowerLimit(self, bcarm_lower_limit_switch):
        self.bcarm_lower_limit_switch = bcarm_lower_limit_switch

  # def getBCForeLimit(self):
      # return self.bc_fore_limit_switch

  # def setBCTransForeLimit(self, bc_fore_limit_switch):
      # self.bc_fore_limit_switch = bc_fore_limit_switch

  # def getBCAltLimit(self):
  #     return self.bc_alt_limit_switch

  # def setBCAltLimit(self, bc_alt_limit_switch):
  #     self.bc_alt_limit_switch = bc_alt_limit_switch

    def getObstacles(self):
        return self.obstacles

    def addObstacle(self, obs):
        if obs not in self.obstacles:
            self.obstacles.append(obs)


#Current position of the robot
currentState = Robot_state()

def simple_drive_test1():
    pass

def simple_drive_test2():
    pass

def simple_drive_test3():
    pass

def simple_turn_test1():
    pass

def simple_turn_test2():
    pass

def simple_turn_test3():
    pass

def transit_test1():
    pass

def transit_test2():
    pass

def transit_test3():
    pass

def deposition_align_test():
    pass

def updateState(msg):
    global currentState

    if msg.sensorID == 0:
        currentState.setPortRPM(msg.value)
    elif msg.sensorID == 1:
        currentState.setStarRPM(msg.value)
    elif msg.sensorID == 9:
        currentState.setDepLoad(msg.value)
    elif msg.sensorID == 11:
        currentState.setGyro0X(msg.value)
    elif msg.sensorID == 12:
        currentState.setGyro0Y(msg.value)
    elif msg.sensorID == 13:
        currentState.setGyro0Z(msg.value)
    elif msg.sensorID == 17:
        currentState.setGyro1X(msg.value)
    elif msg.sensorID == 18:
        currentState.setGyro1Y(msg.value)
    elif msg.sensorID == 19:
        currentState.setGyro1Z(msg.value)
    elif msg.sensorID == 23:
        currentState.setDepLowerLimit(bool(msg.value))
    elif msg.sensorID == 24:
        currentState.setDepUpperLimit(bool(msg.value))
    elif msg.sensorID == 27:
        currentState.setBCArmLowerLimit(bool(msg.value))
    elif msg.sensorID == 28:
        currentState.setBCArmUpperLimit(bool(msg.value))

def updateObstacle(msg):
    global currentState
    obs = pp.Obstacle(msg.x, msg.y, msg.diameter / 2)
    currentState.addObstcle(obs)

def updatePos(msg):
    global currentState
    pos = pp.Position(msg.x, msg.y, msg.theta)
    currentState.setCurrentPos(pos)

def subscribe():
    rospy.Subscriber('sensorValue', sensorValue, updateState)
    rospy.Subscriber('Localization', Pose2D, updatePos)
    rospy.Subscriber('Obstacle', Obstacle, updateObstacle)

def main():
    routines = [simple_drive_test1, simple_drive_test2, simple_drive_test3,
                simple_turn_test1, simple_turn_test2, simple_turn_test3,
                transit_test1, transit_test2, transit_test3, deposition_align_test]
    routine = sys.argv[0]
    rospy.init_node('Auonomy_Test')
    subscribe()
    rospy.on_shutdown(shutdownRoutine)
    rospy.sleep(3)
    routines[routine]()
    rospy.spin()

if __name__ == '__main__':
    main()
