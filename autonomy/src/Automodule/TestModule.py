#!/usr/bin/env python

import rospy
import sys
import time
import math
import PathPlanning.PathPlanning as pp
import DriveControl.MotorCommand as mc

from PathPlanning.PathPublisher import send_path_data
from PathPlanning.ThetaStar import create_path
from apriltags_ros.msg import Localization
from hci.msg import sensorValue, motorCommand
from obstacle_detection.msg import Obstacle


class Robot_state:
    def __init__(self):
        self.currentPos = None
        self.gyro_0z = 0.0
        self.gyro_1z = 0.0
        self.acce_0x = 0.0
        self.acce_1x = 0.0
        self.port_drive_rpm = 0.0
        self.starboard_drive_rpm = 0.0
        self.dep_load_cell = 0.0
        self.dep_lower_limit_switch = False
        self.dep_upper_limit_switch = False
        self.bcarm_lower_limit_switch = False # When translated all the way up
        self.bcarm_upper_limit_switch = False # When translated all the way down
        self.bc_fore_limit_switch = False
        self.bc_alt_limit_switch = False
        self.obstacles = {}
        self.obstacle_found = False

    def getCurrentPos(self):
        return self.currentPos

    def setCurrentPos(self, currentPos):
        self.currentPos = currentPos

    def getGyroZ(self):
        return (self.gyro_0z + self.gyro_1z) /2

    def setGyro0Z(self, gyro_z):
        self.gyro_0z = gyro_z

    def setGyro1Z(self, gyro_z):
        self.gyro_1z = gyro_z

    def getAcceX(self):
        return (self.acce_1x + self.acce_0x) / 2

    def setAcce0X(self, acce_0x):
        self.acce_0x = acce_0x

    def setAcce1X(self, acce_1x):
        self.acce_1x = acce_1x

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

    def getBCForeLimit(self):
       return self.bc_fore_limit_switch

    def setBCTransForeLimit(self, bc_fore_limit_switch):
        self.bc_fore_limit_switch = bc_fore_limit_switch

    def getBCAltLimit(self):
       return self.bc_alt_limit_switch

    def setBCAltLimit(self, bc_alt_limit_switch):
       self.bc_alt_limit_switch = bc_alt_limit_switch

    def getObstacles(self):
        return self.obstacles

    def addObstacle(self, id, obs):
        if id not in self.obstacles.keys():
            self.obstacles[id] = obs
            self.setObstacleFound(True)
            return True
        elif math.fabs(obs.getRadius() - self.obstacles[id].getRadius()) > 0.1:
            self.obstacles[id] = obs
            self.setObstacleFound(True)
            return True
        else:
            return False

    def setObstacleFound(self, found):
        self.obstacle_found = found


#Current position of the robot
currentState = Robot_state()
logfile = None
gyrolog = None
motor_pub = rospy.Publisher('motorCommand', motorCommand, queue_size=100)

ROBOT_SPEED_DRIVE = 20.0
ROBOT_SPEED_DRIVE_LOAD = 30.0
ROBOT_SPEED_TURN = 15.0
ROBOT_SPPED_TURN = 25.0
CONTROL_RATE = 0.005
DECELARATE_RATE = 35 #IN RPM/SEC
WHEEL_RADIUS = 0.2286 # IN M
TURN_RADIUS = 0.3 # IN M
ARENA_WIDTH = 4.2672
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
    gyrolog.write('gyro:' + str(gyro_z) + ' left_rpm:' + str(left_rpm) + ' right_rpm:' + str(right_rpm) + ' cum_angle:' + str(cum_angle) + ' time:' + str(time)+'\n')

def conservative_drive(dest, forward, distance):
    global motor_pub
    done = False
    initPos = currentState.getCurrentPos()
    if distance > 1.85:
        speed = ROBOT_SPEED_DRIVE
    else:
        speed = math.sqrt(distance * 0.5 / 0.0010265)
    stop_distance = distance - 0.001265 * speed ** 2
    while not done:
        if rospy.is_shutdown():
            exit(-1)
        rospy.loginfo('current pos ' + str(currentState.getCurrentPos()))
        rospy.loginfo('distance to destination ' + str(currentState.getCurrentPos().distanceTo(dest)))
        if currentState.getCurrentPos() == dest or\
            initPos.distanceTo(currentState.getCurrentPos()) > stop_distance:
            print 'stopping'
            done = True
        elif math.fabs(initPos.distanceTo(currentState.getCurrentPos())) > distance:
            rospy.loginfo('Did not arrive at destination, but moved far enough')
            rospy.loginfo('deviation: ' + str(dest.distanceTo(currentState.getCurrentPos())))
            done = True
        else:
            if forward:
                mc.drive_left_motor(motor_pub, speed)
                mc.drive_right_motor(motor_pub, speed)
                logDriveData(True, speed)
            else:
                mc.drive_left_motor(motor_pub, -speed)
                mc.drive_right_motor(motor_pub, -speed)
                logDriveData(False, speed)
        if not done:
            next_distance = distance_moved((currentState.getStarRPM() + currentState.getPortRPM()) / 2, currentState.getAcceX(), 0.005)
            if forward:
	        looky_turn(currentState.getCurrentPos(), next_distance)
	    else:
		looky_turn(currentState.getCurrentPos(), -next_distance)

        rospy.sleep(0.005)

    mc.drive_right_motor(motor_pub, 0)
    mc.drive_left_motor(motor_pub, 0)

def turn_algo_2(goal, counter):
    offset = currentState.getGyroZ()
    cum_angle = 0
    stop_angle = 0
    flag = False
    done = False
    initTime = time.time()
    lastTime = None
    goal_orientation = (currentState.getCurrentPos().getOrientation() + toRadian(goal)) % (2 * math.pi)
    goal_pos = pp.Position(currentState.getCurrentPos().getX(), currentState.getCurrentPos().getY(), goal_orientation)
    looky_turn_2(currentState.getCurrentPos(), goal_pos)
    if counter:
        mc.drive_left_motor(motor_pub, -ROBOT_SPEED_TURN)
        mc.drive_right_motor(motor_pub, ROBOT_SPEED_TURN)
    else:
        mc.drive_left_motor(motor_pub, ROBOT_SPEED_TURN)
        mc.drive_right_motor(motor_pub, -ROBOT_SPEED_TURN)

    while not done:
        w = currentState.getGyroZ() - offset
        if not flag:
            if math.fabs(( math.fabs(currentState.getStarRPM())+ math.fabs(currentState.getPortRPM())) / 2 - ROBOT_SPEED_TURN) < 1:
                print 'hi'
                stop_angle = goal - cum_angle
                print str(stop_angle)
                flag = True
            elif cum_angle >= goal / 2:
                stop_angle = cum_angle
                flag = True

        if lastTime is None:
            lastTime = time.time()
        else:
            currentTime = time.time()
            deltaT = currentTime - lastTime
            cum_angle += math.fabs(w) * deltaT
            lastTime = currentTime
            logGyroData(currentState.getGyroZ() + offset, currentState.getPortRPM(), currentState.getStarRPM(), cum_angle, currentTime - initTime)

        if flag and cum_angle >= stop_angle:
            mc.drive_right_motor(motor_pub, 0)
            mc.drive_left_motor(motor_pub, 0)
            done = True

        if rospy.is_shutdown():
            exit(-1)
    while currentState.getGyroZ() > 1:
        w = currentState.getGyroZ() - offset
        currentTime = time.time()
        cum_angle += math.fabs(w  *  (currentTime - lastTime))
        lastTime = currentTime
        rospy.sleep(0.005)
    measure = raw_input('How much did it turn? (in deg): ')
    logTurnData(counter, ROBOT_SPEED_TURN, goal, cum_angle, float(measure), lastTime - initTime)

def convert_axis(pos, dx, dy):
    return pp.Position(pos.getY() + dy, dx - pos.getX(), (pos.getOrientation() + 3 * math.pi / 2) % (2 * math.pi))

DELTA_X = 1.23
DELTA_Y = 0.825


"""
ID 0 : Port
ID 1 : Star
ID 2 : Back
"""
def looky_turn(currentPos, next_distance):
    nextX = currentPos.getX() + next_distance * math.cos(currentPos.getOrientation())
    nextY = currentPos.getY() + next_distance * math.sin(currentPos.getOrientation())
    nextPos = pp.Position(nextX, nextY, currentPos.getOrientation())
    looky_angle = (nextPos.angleToFace(pp.Position(0,0,0))) % (2 * math.pi)
    if looky_angle >= (toRadian(-150) % 2 * math.pi) or looky_angle <= toRadian(150):
        if looky_angle <= toRadian(150):
            mc.star_looky(motor_pub, toDegree(looky_angle))
        else:
            mc.star_looky(motor_pub, toDegree(looky_angle - 2*math.pi))
    elif looky_angle >= ((toRadian(-150) + math.pi) % (2 * math.pi)) \
        and looky_angle <= ((toRadian(150) + math.pi) % (2 * math.pi)):
        mc.port_looky(motor_pub, toDegree(looky_angle - math.pi))
    else:
        mc.star_looky(motor_pub, 150)
        mc.port_looky(motor_pub, -150)

def looky_turn_2(currentPos, next_pos):
    looky_angle = (currentPos.angleToFace(pp.Position(0,0,0))) % (2 * math.pi)
    print str(looky_angle)
    if looky_angle >= (toRadian(-150) % (2 * math.pi)) or looky_angle <= toRadian(150):
        if looky_angle <= toRadian(150):
            mc.star_looky(motor_pub, toDegree(looky_angle))
        else:
            mc.star_looky(motor_pub, toDegree(looky_angle - 2 * math.pi))
    elif looky_angle >= ((toRadian(-150) + math.pi) % (2 * math.pi)) \
            and looky_angle <= ((toRadian(150) + math.pi) % (2 * math.pi)):
        mc.port_looky(motor_pub, toDegree(looky_angle - math.pi))
    else:
        mc.star_looky(motor_pub, 150)
        mc.port_looky(motor_pub, -150)

def angle_moved(angular_velocity, t):
    print 'time ' +  str(t)
    return math.fabs(toRadian(angular_velocity) * t / 0.3)

def distance_moved(rpm, acce, t):
    rps = rpm / 60
    speed = rps * WHEEL_RADIUS * 2 * math.pi
    distance = math.fabs(speed) * t + 0.5 * acce * t ** 2
    return math.fabs(distance)

def toRadian(deg):
    return deg * math.pi / 180

def simple_drive_test1():
    rospy.loginfo('This test routine attempts to drive straight from is current position')
    distance = float(raw_input('Enter distance you want to travel'))
    while True:
        if rospy.is_shutdown():
            exit(-1)
        print 'current position: ' + str(currentState.getCurrentPos())
        answer = raw_input('Make sure that robot is in a place okay to drive forward. Start testing?  y/n ')
        if answer == 'y':
            break

    currentPos = currentState.getCurrentPos()
    destination = pp.Position(currentPos.getX() + distance * math.cos(currentPos.getOrientation()),
                              currentPos.getY() + distance * math.sin(currentPos.getOrientation()),
                              currentPos.getOrientation())

    direction = True
    if distance > 0 :
        direction = True
    else:
        direction = False;

    conservative_drive(destination, direction, math.fabs(distance))
    measure = float(raw_input('How much did it move? (in m): '))
    rospy.loginfo('distance_travelled:' + str(measure) + ' distance_entered:' + str(distance))
    print 'testing successful'
    exit(0)

def simple_drive_test2():
    print 'testing tracking lookys while driving'

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
    goal_in_rad = toRadian(goal)
    currentPos = currentState.getCurrentPos()
    destination = pp.Position(currentPos.getX(), currentPos.getY(),
                              (currentPos.getOrientation() + goal_in_rad) % (2 * math.pi))
    direction = True
    if goal > 0:
        direction = True
    else:
        direction = False

    turn_algo_2(math.fabs(goal), direction)
    print 'testing succesful'
    rospy.loginfo('localization data:')
    rospy.loginfo('start_angle:' + str(toDegree(currentPos.getOrientation())) + ' end_angle' + str(toDegree(currentState.getCurrentPos().getOrientation()))+ ' actual_goal:' + str(toDegree(destination.getOrientation())))
    exit(0)

def toDegree(rad):
    return rad * 180 / math.pi

def simple_turn_test2():
    rospy.loginfo('This test routine attempts to turn in place given degrees')
    while True:
        print 'current position: ' + str(currentState.getCurrentPos())
        answer = raw_input('Start testing?  y/n ')
        if answer == 'y':
            break
    goal = float(raw_input('Enter how much to turn in degrees. input should be less than 360'))
    direction = True
    if goal > 0:
        direction = True
    else:
        direction = False
    turn_algo_2(math.fabs(goal), direction)
    print 'testing successful'
    exit(0)

def simple_turn_test3():
    print 'not implemented'
    exit(0)

def convertToCommands(path):
    commands = []
    for position in path.path:
        currentPos = currentState.getCurrentPos()
        position.x_pos = ARENA_WIDTH - position.getX();
        angle_to_face = currentPos.angleToFace(position)
        angle_turn = angle_to_face - currentPos.getOrientation()
        distance = currentPos.distanceTo(position)
        commands.append((angle_turn, distance))
    return commands

def transit_test1():
    print 'simple path following stuff'
    print 'currentPos: ' + str(currentState.getCurrentPos())
    dest_x = float(raw_input('enter destination x pos'))
    dest_y = float(raw_input('enter destination y pos'))
    dest = pp.Position(dest_x, dest_y)
    print str(len(currentState.getObstacles().values()))
    path = create_path(currentState.getCurrentPos(), dest, 4.2672, 6.096, currentState.getObstacles().values())
    send_path_data(path)
    for position in path.path:
	print str(position)

    exit(0)
    commands = convertToCommands(path)
    for command in commands:
        if command[0] > 0:
            turn_algo_2(command[0], True)
        else:
            turn_algo_2(math.fabs(command[0]), False)
        dest_x = currentState.getCurrentPos().getX() + command[1] * math.cos(currentState.getCurrentPos().getOrientation())
        dest_y = currentState.getCurrentPos().getY() + command[1] * math.sin(currentState.getCurrentPos().getOrientation())
        dest = pp.Position(dest_x, dest_y, currentState.getCurrentPos().getOrientation())
        if command[1] > 0:
            conservative_drive(dest, True, command[1])
        else:
            conservative_drive(dest, False, math.fabs(command[1]))
    print 'testing done'
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
    elif msg.sensorID == 13:
        currentState.setGyro0Z(msg.value)
    elif msg.sensorID == 14:
        pass
    elif msg.sensorID == 19:
        currentState.setGyro1Z(msg.value)
    elif msg.sensorID == 20:
        pass
#    elif msg.sensorID == 23:
 #       currentState.setDepLowerLimit(bool(msg.value))
 #   elif msg.sensorID == 24:
 #       currentState.setDepUpperLimit(bool(msg.value))
 #   elif msg.sensorID == 27:
 #       currentState.setBCArmLowerLimit(bool(msg.value))
 #   elif msg.sensorID == 28:
 #       currentState.setBCArmUpperLimit(bool(msg.value))

def updateObstacle(msg):
    global currentState
    obs = pp.Obstacle(msg.x, msg.y, msg.diameter / 2)
    if currentState.addObstacle(msg.obsID, obs):
	print 'added obstacle'
        mc.drive_left_motor(motor_pub, 0)
        mc.drive_right_motor(motor_pub, 0)

def updatePos(msg):
    global currentState
    pos = pp.Position(msg.x, msg.y, msg.theta)
    currentState.setCurrentPos(pos)

def subscribe():
    rospy.Subscriber('sensorValue', sensorValue, updateState)
    rospy.Subscriber('localization_data', Localization, updatePos)
    rospy.Subscriber('obstacleDetection', Obstacle, updateObstacle)

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
    print 'waiting on localization data'
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
