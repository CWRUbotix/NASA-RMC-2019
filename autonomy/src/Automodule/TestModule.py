#!/usr/bin/env python

import rospy
import sys
import time
import math
import PathPlanning.PathPlanning as pp
import DriveControl.MotorCommand as mc

from geometry_msgs.msg import Pose2D
from hci.msg import sensorValue, motorCommand
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
gyrolog = None
motor_pub = rospy.Publisher('motorCommand', motorCommand, queue_size=100)

ROBOT_SPEED_DRIVE = 30.0
ROBOT_SPEED_TURN = 10.0
CONTROL_RATE = 0.005
DECELARATE_RATE = 35 #IN RPM/SEC
#15 rpm, 0.4m, 30 rpm, 0.6m
def logDriveData(direction, value):
    log = ''
    if direction:
        log = 'Drive:Forward Value:' + str(value)
    else:
        log = 'Drive:Backward Value:' + str(value)

    log += ' Left RPM:' + str(currentState.getPortRPM()) + ' right RPM:' + str(currentState.getStarRPM())
    logfile.write(log + '\n')

def logTurnData(direction, value, goal, theo_angle_moved, actual_angle_moved, dt):
    if direction:
        log = 'Turn:Counter Value:' + str(value)
    else:
        log = 'Turn:Clockwise Value:' + str(value)

    log += 'goal:' + str(goal) + ' angle_calculated:' + str(theo_angle_moved) + ' angle_moved:' + str(actual_angle_moved) + ' delta_t:' + str(dt)
    logfile.write(log + '\n')

def logGyroData(gyro_z, left_rpm, right_rpm, cum_angle, time):
    gyrolog.write('gyro:' + str(gyro_z) + ' left_rpm:' + str(left_rpm) + ' right_rpm:' + str(right_rpm) + ' cum_angle:' + str(cum_angle) + ' time:' + str(time))

def conservative_drive(dest, forward, distance, deceleration):
    global motor_pub
    done = False
    initPos = currentState.getCurrentPos()
    speed = ROBOT_SPEED_DRIVE

    while not done:
        if rospy.is_shutdown():
            exit(-1)
        rospy.loginfo('current pos ' + str(currentState.getCurrentPos()))
        rospy.loginfo('distance to destination ' + str(currentState.getCurrentPos().distanceTo(dest)))
        if currentState.getCurrentPos() == dest:
            done = True
        elif math.fabs(initPos.distanceTo(currentState.getCurrentPos())) > distance:
            rospy.loginfo('Did not arrive at destination, but moved far enough')
            rospy.loginfo('deviation: ' + str(dest.distanceTo(currentState.getCurrentPos())))
            done = True
            exit(-1)
        else:
            if math.fabs(initPos.distanceTo(currentState.getCurrentPos())) > distance * 0.75    :
                speed -= speed * deceleration
                speed = max(0, speed)

            if forward:
                mc.drive_left_motor(motor_pub, speed)
                mc.drive_right_motor(motor_pub, speed)
                logDriveData(True, speed)
            else:
                mc.drive_left_motor(motor_pub, -speed)
                mc.drive_right_motor(motor_pub, -speed)
                logDriveData(False, speed)
            print 'sent motorCommand'
        rospy.sleep(0.005)

    mc.drive_right_motor(motor_pub, 0)
    mc.drive_left_motor(motor_pub, 0)

def conservative_turn(goal, counter, deceleration):
    global motor_pub
    done = False
    r = rospy.Rate(1/CONTROL_RATE)
    cum_angle_turn = 0
    lastTime = None
    initTime = time.time()
    speed = ROBOT_SPEED_TURN
    while not (done or rospy.is_shutdown()):
        rospy.loginfo('Gyro:' + str(currentState.getGyroZ()))
        currentTime = time.time()
        logGyroData(currentState.getGyroZ(), currentState.getPortRPM(), currentState.getStarRPM(), cum_angle_turn, currentTime - initTime)
        if lastTime is not None:
            cum_angle_turn += angle_moved(currentState.getGyroZ(), currentTime - lastTime)
            cum_angle_turn = min(cum_angle_turn, goal)
        print str(cum_angle_turn)
        rospy.loginfo('cum_angle:' + str(cum_angle_turn) + ' time:' + str(currentTime - initTime))
        angleLeft = goal - cum_angle_turn
        if angleLeft < 0.25 * goal:
            speed -= speed * deceleration
            speed = max(0, speed)
        if counter:
            mc.drive_left_motor(motor_pub, -speed)
            mc.drive_right_motor(motor_pub, speed)
        else:
            mc.drive_left_motor(motor_pub, speed)
            mc.drive_right_motor(motor_pub, -speed)
        lastTime = currentTime
        if math.fabs(goal - cum_angle_turn) < 5 or cum_angle_turn > goal:
            done = True
        r.sleep()
    mc.drive_right_motor(motor_pub, 0)
    mc.drive_left_motor(motor_pub, 0)
    if rospy.is_shutdown():
        exit(-1)
    measure = raw_input('How much did it turn? (in deg): ')
    logTurnData(counter, ROBOT_SPEED_TURN, goal, cum_angle_turn, float(measure), lastTime - initTime)

def angle_moved(angular_velocity, t):
    print 'time ' +  str(t)
    return math.fabs(angular_velocity * t)

def toRadian(deg):
    return deg * math.pi / 180

def simple_drive_test1():
    rospy.loginfo('This test routine attempts to drive straight from is current position')
    distance = float(raw_input('Enter distance you want to travel'))
    deceleration = float(raw_input('Enter deceleration factor you want'))
    while True:
        if rospy.is_shutdown():
            exit(-1)
        print 'current position: ' + str(currentState.getCurrentPos())
        answer = raw_input('Make sure that robot is in a place okay to drive forward. Start testing?  y/n ')
        if answer == 'y':
            break

    currentPos = currentState.getCurrentPos()
    destination = pp.Position(currentPos.getX_pos() + distance * math.cos(currentPos.getOrientation()),
                              currentPos.getY_pos() + distance * math.sin(currentPos.getOrientation()),
                              currentPos.getOrientation())

    direction = True
    if distance > 0 :
        direction = True
    else:
        direction = False;

    conservative_drive(destination, direction, math.fabs(distance), deceleration)
    measure = float(raw_input('How much did it move? (in m): '))
    rospy.loginfo('distance_travelled:' + str(measure) + ' distance_entered:' + str(distance))
    print 'testing successful'
    exit(0)

def simple_drive_test2():
    print 'testing not implemented'
    exit(0)

def simple_drive_test3():
    print 'testing not implemented'
    exit(0)

def simple_turn_test1():
    rospy.loginfo('This test routine attempts to turn in place given degrees')
    while True:
        print 'current position: ' + str(currentState.getCurrentPos())
        answer = raw_input('Start testing?  y/n ')
        if answer == 'y':
            break
    goal = float(raw_input('Enter how much to turn in degrees. input should be less than 360'))
    deceleration = float(raw_input('Enter deceleration factor you want'))
    goal_in_rad = toRadian(goal)
    currentPos = currentState.getCurrentPos()
    destination = pp.Position(currentPos.getX_pos(), currentPos.getY_pos(),
                              (currentPos.getOrientation() + goal_in_rad) % (2 * math.pi))
    direction = True
    if goal < 0:
        direction = True
    else:
        direction = False

    conservative_turn(math.fabs(goal), direction, deceleration)
    print 'testing succesful'
    rospy.loginfo('localization data:')
    rospy.loginfo('start_angle:' + str(toDegree(currentPos.getOrientation())) + ' end_angle' + str(toDegree(currentState.getCurrentPos().getOrientation()))+ ' actual_goal:' + str(toDegree(destination.getOrientation())))
    exit(0)

def toDegree(rad):
    return rad * 180 / math.pi

def simple_turn_test2():
    print 'not implemented'
    exit(0)

def simple_turn_test3():
    print 'not implemented'
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
    print 'waiting localization data'
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
   # elif msg.sensorID == 23:
     #   currentState.setDepLowerLimit(bool(msg.value))
    #elif msg.sensorID == 24:
     #   currentState.setDepUpperLimit(bool(msg.value))
    #elif msg.sensorID == 27:
     #   currentState.setBCArmLowerLimit(bool(msg.value))
    #elif msg.sensorID == 28:
     #   currentState.setBCArmUpperLimit(bool(msg.value))

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
    rospy.Subscriber('localization_data', Pose2D, updatePos)
    rospy.Subscriber('Obstacle', Obstacle, updateObstacle)

def testShutdown():
    global logfile, motor_pub
    mc.drive_left_motor(motor_pub, 0)
    mc.drive_right_motor(motor_pub, 0)
    logfile.close()

def main():
    print "setting up logging"
    global logfile, gyrolog
    logfile = open(str(sys.path[1]) + '/logs/' +time.strftime("%b-%a-%d-%H-%M-%S.txt"), "w")
    gyrolog = open(str(sys.path[1]) + '/logs/gyro_' + time.strftime("%b-%a-%d-%H-%M-%S.txt"), 'w')

    routines = [simple_drive_test1, simple_drive_test2, simple_drive_test3,
                simple_turn_test1, simple_turn_test2, simple_turn_test3,
                transit_test1, transit_test2, transit_test3, deposition_align_test]
    routine = int(sys.argv[1])
    rospy.init_node('Autonomy_Test')
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
