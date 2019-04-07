#!/usr/bin/env python

import rospy
import sys
import time
import math
import PathPlanning.PathPlanning as pp
import DriveControl.MotorCommand as mc

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
logfile = None
ROBOT_SPEED = 0.0

def logData(drive, direction, value):
    log = ""
    if drive:
        if direction:
            log = 'Drive, Forward, Value: ' + str(value)
        else:
            log = 'Drive, Backward, Value: ' + str(value)
    else:
        if direction:
            log = 'Turn, Counter, Value: ' + str(value)
        else:
            log = 'Turn, Clockwise, Value: ' + str(value)

    log += ' Gyro_x: ' + str(currentState.getGyroX()) + 'Gyro_y:' + str(currentState.getGyroY()) + 'Gyro_z' + str(currentState.getGyroZ())
    logfile.write(log + '\n')

def conservative_drive(dest, forward):
    global logfile
    done = False
    initPos = currentState.getCurrentPos()
    while not done:
        print 'current pos ' + str(currentState.getCurrentPos())
        print 'distance to destination ' + str(currentState.getCurrentPos().distanceTo(dest))
        if currentState.getCurrentPos() == dest:
            done = True
        elif math.fabs(initPos.distanceTo(currentState.getCurrentPos()) - initPos.distanceTo(dest)) < 0.05:
            print 'Did not arrive at destination, but moved far enough'
            print 'deviation: ' + str(dest.distanceTo(currentState.getCurrentPos()))
            done = True
            exit(-1)
        else:
            speed = ROBOT_SPEED
            if dest.distanceTo(currentState.getCurrentPos()) < 0.2:
                speed = 0.5 * speed

            if forward:
                mc.drive_left_motor(speed)
                mc.drive_right_motor(speed)
                logData(True, True, speed)
            else:
                mc.drive_left_motor(-speed)
                mc.drive_right_motor(-speed)
                logData(True, False, speed)
        rospy.sleep(0.005)

def conservative_turn(dest, clockwise):
    done = False
    while not done:
        print 'current angle in degrees: ' + str((currentState.getCurrentPos().getOrientation())/math.pi*180)
        print 'turns left in degrees: ' + str((currentState.getCurrentPos().angleToFace(dest))/math.pi*180)

        if currentState.getCurrentPos().angleToFace(dest) < math.pi/ 180 * 5: #5 degrees within
            done = True
        else:
            speed = ROBOT_SPEED

            #when within 25 degrees, slow down a bit
            if currentState.getCurrentPos().angleToFace(dest) < math.pi / 180 * 25:
                speed = 0.5 * speed
            if clockwise:
                mc.drive_left_motor(speed)
                mc.drive_right_motor(-speed)
                logData(False, False, speed)
            else:
                mc.drive_left_motor(-speed)
                mc.drive_right_motor(speed)
                logData(False, True, speed)
        rospy.sleep(0.005)

def simple_drive_test1():
    print 'This test routine attempts to drive 1.5 meter straight from is current position'

    while True:
        print 'current position: ' + str(currentState.getCurrentPos())
        answer = raw_input('Make sure that robot is in a place okay to drive 1 meter forward. Start testing?  y/n ')
        if answer == 'y':
            break

    currentPos = currentState.getCurrentPos()
    destination = pp.Position(currentPos.getX_pos() + 1.5 * math.cos(currentPos.getOrientation()),
                              currentPos.getY_pos() + 1.5 * math.sin(currentPos.getOrientation()),
                              currentPos.getOrientation())

    conservative_drive(destination, True)
    print 'testing successful'
    exit(0)

def simple_drive_test2():
    print 'This test routine attempts to drive 1.5 meter backward from is current position'

    while True:
        print 'current position: ' + str(currentState.getCurrentPos())
        answer = raw_input('Make sure that robot is in a place okay to drive 1 meter forward. Start testing?  y/n ')
        if answer == 'y':
            break

    currentPos = currentState.getCurrentPos()
    destination = pp.Position(currentPos.getX_pos() - 1.5 * math.cos(currentPos.getOrientation()),
                              currentPos.getY_pos() - 1.5 * math.sin(currentPos.getOrientation()),
                              currentPos.getOrientation())

    conservative_drive(destination, False)
    print 'testing successful'
    exit(0)

def simple_drive_test3():
    print 'This test routine attempts to drive 1.5 meter forward and backward from is current position'

    while True:
        print 'current position: ' + str(currentState.getCurrentPos())
        answer = raw_input('Make sure that robot is in a place okay to drive 1 meter forward. Start testing?  y/n ')
        if answer == 'y':
            break

    currentPos = currentState.getCurrentPos()
    destination = pp.Position(currentPos.getX_pos() + 1.5 * math.cos(currentPos.getOrientation()),
                              currentPos.getY_pos() + 1.5 * math.sin(currentPos.getOrientation()),
                              currentPos.getOrientation())

    conservative_drive(destination, True)
    currentPos = currentState.getCurrentPos()
    destination = pp.Position(currentPos.getX_pos() - 1.5 *math.cos(currentPos.getOrientation()),
                              currentPos.getY_pos() - 1.5 * math.sin(currentPos.getOrientation()),
                              currentPos.getOrientation())
    conservative_drive(destination, False)
    print 'testing successful'
    exit(0)

def simple_turn_test1():
    print 'This test routine attempts to turn in place given degrees counter-clockwise'
    while True:
        print 'current position: ' + str(currentState.getCurrentPos())
        answer = raw_input('Start testing?  y/n ')
        if answer == 'y':
            break
    goal = float(raw_input('Enter how much to turn in degrees. input should be less than 360'))
    goal = goal * math.pi / 180.0
    currentPos = currentState.getCurrentPos()
    destination = pp.Position(currentPos.getX_pos(), currentPos.getY_pos(),
                              (currentPos.getOrientation() + goal) % (2 * math.pi))
    conservative_turn(destination, False)
    print 'testing succesful'
    exit(0)

def simple_turn_test2():
    print 'This test routine attempts to turn in place given degrees clockwise'
    while True:
        print 'current position: ' + str(currentState.getCurrentPos())
        answer = raw_input('Start testing?  y/n ')
        if answer == 'y':
            break
    goal = float(raw_input('Enter how much to turn in degrees. input should be less than 360'))
    goal = goal * math.pi / 180.0
    currentPos = currentState.getCurrentPos()
    destination = pp.Position(currentPos.getX_pos(), currentPos.getY_pos(),
                              (currentPos.getOrientation() - goal) % (2 * math.pi))
    conservative_turn(destination, True)
    print 'testing succesful'
    exit(0)

def simple_turn_test3():
    print 'This test routine attempts to turn in place given degrees counter-clockwise and then clockwise'
    while True:
        print 'current position: ' + str(currentState.getCurrentPos())
        answer = raw_input('Start testing?  y/n ')
        if answer == 'y':
            break
    goal = float(raw_input('Enter how much to turn in degrees. input should be less than 360'))
    goal = goal * math.pi / 180.0
    currentPos = currentState.getCurrentPos()
    destination = pp.Position(currentPos.getX_pos(), currentPos.getY_pos(),
                              (currentPos.getOrientation() + goal) % (2 * math.pi))
    conservative_turn(destination, False)

    currentPos = currentState.getCurrentPos()
    destination = pp.Position(currentPos.getX_pos(), currentPos.getY_pos(),
                              (currentPos.getOrientation() - goal) % (2 * math.pi))
    conservative_turn(destination, True)
    print 'testing succesful'
    exit(0)

def transit_test1():
    print 'trasit_Test1 not ready yet'
    exit(0)

def transit_test2():
    print 'trasit_Test2 not ready yet'
    exit(0)

def transit_test3():
    print 'trasit_Test3 not ready yet'
    exit(0)

def deposition_align_test():
    print 'align_test not ready yet'
    exit(0)

def waitForLocalization():
    while currentState.getCurrentPos() is None:
        rospy.sleep(0.01)

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

def testShutdown():
    global logfile
    mc.drive_left_motor(0)
    mc.drive_right_motor(0)
    logfile.close()

def main():
    print sys.path[2]
    print "setting up logging"
    global logfile
    logfile = open(str(sys.path[1]) + '/logs/' +time.strftime("%b-%a-%d-%H-%M-%S.txt"), "w")

    routines = [simple_drive_test1, simple_drive_test2, simple_drive_test3,
                simple_turn_test1, simple_turn_test2, simple_turn_test3,
                transit_test1, transit_test2, transit_test3, deposition_align_test]
    routine = int(sys.argv[1])
    rospy.init_node('Auonomy_Test')
    subscribe()
    rospy.on_shutdown(testShutdown)
    waitForLocalization()
    print "Testing ready"
    print "Routine " + str(sys.argv[1])
    while True:
        answer = raw_input("Start testing? y/n ")
        if answer == 'y':
            break
    routines[routine]()
    rospy.spin()

if __name__ == '__main__':
    main()
