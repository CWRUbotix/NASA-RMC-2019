#!/usr/bin/env python

import rospy
import math
import time
import sys
import DriveControl.MotorCommand as mc
import PathPlanning.PathPlanning as pp

from PathPlanning.ThetaStar import create_path
# from PathPlanning.PathTesting import drawPath
from RobotState import Robot_state
from apriltags_ros.msg import Localization
from hci.msg import sensorValue, motorCommand
from obstacle_detection.msg import Obstacle
from std_msgs.msg import Float32

""" 
Main Module for Autonomy operation
"""

"""
Constants
"""
ARENA_WIDTH = 3.0
ARENA_HEIGHT = 7.0
COLLECTION_BIN = pp.Position(0, 0)
ROBOT_SPEED_DRIVE = 20.0
ROBOT_SPEED_DRIVE_LOAD = 30.0
ROBOT_SPEED_TURN = 15.0
ROBOT_SPPED_TURN_LOAD = 25.0
CONTROL_RATE = 0.005
WHEEL_RADIUS = 0.2286  # IN M
TURN_RADIUS = 0.3  # IN M
# Map from ID to time allocated.
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

"""
Global variables
"""
currentState = Robot_state()
motor_pub = rospy.Publisher('motorCommand', motorCommand, queue_size=100)
logfile = None

"""Routine Builder Stuff"""

"""
Takes in task ID and create list of function calls that
"""


def runRoutine(ID):
    pass


"""
Functions that belong to specific tasks
naming: task_<ID>
Should not be taking in inputs. if you need something, make it global
"""


def task_1():
    pass


# Cycle 1 digging goal : 100
def task_4():
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


# Dumping (Cycle1): 15
def task_8():
    # Need to check the actual angle depending on the incline of the bucket
    angleStrength = 225;
    mc.bucket_angle_actuator(angleStrength)
    rospy.sleep(7)
    mc.bucket_angle_actuator(-angleStrength)
    rospy.sleep(7)


# Cycle 2 digging goal : 100
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


# Dumping (Cycle2): 15
def task_15():
    angleStrength = 225
    mc.bucket_angle_actuator(angleStrength)
    rospy.sleep(7)
    mc.bucket_angle_actuator(-angleStrength)
    rospy.sleep(7)


def modifyTimes(timeLeft):
    pass


"""
Control Related functions
"""


def updateRelativePos(d_distance, d_angle):
    pos = currentState.getCurrentPos()
    if d_distance > 0:
        x = pos.getX() + d_distance * math.cos(pos.getOrientation())
        y = pos.getY() + d_distance * math.sin(pos.getOrientation())
        currentState.setCurrentPos(pp.Position(x, y, pos.getOrientation()))
    elif d_angle > 0:
        next_pos = pp.Position(pos.getX(), pos.getY(), (pos.getOrientation() + toRadian(d_angle)) % (2 * math.pi))
        currentState.setCurrentPos(next_pos)


def rpmdrive(dest, forward, distance, speed):
    global motor_pub, currentState
    offset = math.fabs(currentState.getStarRPM() + currentState.getPortRPM()) / 2
    done = False
    flag = False
    cum_distance = 0
    stop_dist = 0
    lastTime = None
    currentState.setDisp(True)
    if forward:
        mc.drive_left_motor(motor_pub, speed)
        mc.drive_right_motor(motor_pub, speed)
    else:
        mc.drive_right_motor(motor_pub, -speed)
        mc.drive_left_motor(motor_pub, -speed)
    while not done:
        rpm = math.fabs(currentState.getPortRPM() + currentState.getStarRPM()) / 2 - offset
        if not flag:
            if rpm - speed < 2.5:
                stop_dist = distance - cum_distance - 0.3
                flag = True
            elif cum_distance >= distance:
                stop_dist = cum_distance
                flag = True
        if lastTime is None:
            lastTime = time.time()
        else:
            currentTime = time.time()
            delta = currentTime - lastTime
            cum_distance += distance_moved(rpm, 0, delta)
            updateRelativePos(distance_moved(rpm, 0, delta), 0)
            lastTime = currentTime
        if flag and cum_distance >= stop_dist:
            mc.drive_left_motor(motor_pub, 0)
            mc.drive_right_motor(motor_pub, 0)
            done = True
        if rospy.is_shutdown():
            exit(-1)
        if currentState.obstacle_found:
            print 'found'
            return False
        data = 0
        if forward:
            data = math.fabs(distance)
        else:
            data = -math.fabs(distance)
        logData('d', rpm, currentState.getAcceX(), currentState.getGyroZ(), data, 0, currentState.getCurrentPos())
        rospy.sleep(0.005)

    rpm = (currentState.getPortRPM() + currentState.getStarRPM()) / 2

    while math.fabs(rpm) > 1:
        currentTime = time.time()
        delta = currentTime - lastTime
        updateRelativePos(distance_moved(rpm, 0, delta), 0)
        lastTime = currentTime
        data = 0
        if forward:
            data = math.fabs(distance)
        else:
            data = -math.fabs(distance)
        logData('d', rpm, currentState.getAcceX(), currentState.getGyroZ(), data, 0, currentState.getCurrentPos())
        rpm = (currentState.getPortRPM() + currentState.getStarRPM()) / 2

    rospy.loginfo("optical displacement: " + str(currentState.getDist()))
    currentState.setDisp(False)
    looky_turn_2(dest, COLLECTION_BIN)
    rospy.sleep(1)
    return True


def drive(dest, forward, distance, speed):
    global motor_pub
    offset = math.fabs(currentState.getStarRPM() + currentState.getPortRPM()) / 2
    done = False
    flag = False
    cum_distance = 0
    stop_dist = 0
    lastTime = None
    result = True
    currentState.setDisp(True)
    if forward:
        mc.drive_left_motor(motor_pub, speed)
        mc.drive_right_motor(motor_pub, speed)
    else:
        mc.drive_right_motor(motor_pub, -speed)
        mc.drive_left_motor(motor_pub, -speed)
    while not done:
        rpm = math.fabs(currentState.getPortRPM() + currentState.getStarRPM()) / 2 - offset
        if not flag:
            if rpm - speed < 2.5:
                stop_dist = distance - cum_distance - 0.3
                flag = True
            elif cum_distance >= distance:
                stop_dist = cum_distance
                flag = True
        if lastTime is None:
            lastTime = time.time()
        else:
            currentTime = time.time()
            delta = currentTime - lastTime
            cum_distance += distance_moved(rpm, 0, delta)
            lastTime = currentTime
        if flag and cum_distance >= stop_dist:
            mc.drive_left_motor(motor_pub, 0)
            mc.drive_right_motor(motor_pub, 0)
            done = True
        if rospy.is_shutdown():
            exit(-1)
        if currentState.obstacle_found:
            mc.drive_left_motor(motor_pub, 0)
            mc.drive_right_motor(motor_pub, 0)
            done = True
            result = False
        rospy.sleep(0.005)
    rospy.loginfo("optical displacement: " + str(currentState.getDist()))
    currentState.setDisp(False)
    looky_turn_2(dest, COLLECTION_BIN)
    rospy.sleep(1)
    return result



def turn(goal, counter, speed):
    offset = currentState.getGyroZ()
    cum_angle = 0
    stop_angle = 0
    flag = False
    done = False
    lastTime = None
    if counter:
        mc.drive_left_motor(motor_pub, -speed)
        mc.drive_right_motor(motor_pub, speed)
    else:
        mc.drive_left_motor(motor_pub, speed)
        mc.drive_right_motor(motor_pub, -speed)

    result = True

    while not done:
        w = currentState.getGyroZ() - offset
        if not flag:
            if math.fabs((math.fabs(currentState.getStarRPM()) + math.fabs(
                    currentState.getPortRPM())) / 2 - speed) < 1:
                stop_angle = goal - cum_angle
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
            updateRelativePos(0, toRadian(w * deltaT))
            lastTime = currentTime

        if flag and cum_angle >= stop_angle:
            mc.drive_right_motor(motor_pub, 0)
            mc.drive_left_motor(motor_pub, 0)
            done = True

        if rospy.is_shutdown():
            exit(-1)

        if currentState.obstacle_found:
            mc.drive_right_motor(motor_pub, 0)
            mc.drive_left_motor(motor_pub, 0)
            done = True
            result = False

        data = 0
        rpm = (math.fabs(currentState.getPortRPM()) + math.fabs(currentState.getStarRPM())) / 2
        if counter:
            data = math.fabs(goal)
        else:
            data = -math.fabs(goal)
        logData('t', rpm, currentState.getAcceX(), currentState.getGyroZ(), 0, data, currentState.getCurrentPos())

        data = 0
        rpm = (math.fabs(currentState.getPortRPM()) + math.fabs(currentState.getStarRPM())) / 2
        if counter:
            data = math.fabs(goal)
        else:
            data = -math.fabs(goal)
        logData('t', rpm, currentState.getAcceX(), currentState.getGyroZ(), 0, data, currentState.getCurrentPos())

        rospy.sleep(0.005)

    rpm = (math.fabs(currentState.getPortRPM()) + math.fabs(currentState.getStarRPM())) / 2
    while math.fabs(rpm) > 1:
        w = currentState.getGyroZ() - offset
        currentTime = time.time()
        deltaT = currentTime - lastTime
        updateRelativePos(0, toRadian(w * deltaT))
        lastTime = currentTime
        data = 0
        if counter:
            data = math.fabs(goal)
        else:
            data = -math.fabs(goal)
        logData('t', rpm, currentState.getAcceX(), currentState.getGyroZ(), 0, data, currentState.getCurrentPos())

        rpm = (math.fabs(currentState.getPortRPM()) + math.fabs(currentState.getStarRPM())) / 2

    looky_turn_2(currentState.getCurrentPos(), COLLECTION_BIN)
    rospy.sleep(1)
    return result



def looky_turn(currentPos, next_distance):
    nextX = currentPos.getX() + next_distance * math.cos(currentPos.getOrientation())
    nextY = currentPos.getY() + next_distance * math.sin(currentPos.getOrientation())
    nextPos = pp.Position(nextX, nextY, currentPos.getOrientation())
    looky_turn_2(nextPos, COLLECTION_BIN)


def looky_turn_2(currentPos, next_pos):
    looky_angle = (currentPos.angleToFace(next_pos) - currentPos.getOrientation() + 2 * math.pi) % (2 * math.pi)

    if looky_angle >= toRadian(360 - 150) or looky_angle <= toRadian(150):
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
    return math.fabs(toRadian(angular_velocity) * t / 0.3)


def distance_moved(rpm, acce, t):
    rps = rpm / 60
    speed = rps * WHEEL_RADIUS * 2 * math.pi
    distance = speed * t + 0.5 * acce * t ** 2
    return math.fabs(distance)


def toDegree(rad):
    return rad * 180 / math.pi


def toRadian(deg):
    return deg * math.pi / 180


def converToCommands(path):
    commands = []
    first_pos = True
    currentPos = currentState.getCurrentPos()
    for position in path.path:
        if first_pos:
            first_pos = False
        else:
            angle_to_face = currentPos.angleToFace(position)
            pos = pp.Position(currentPos.getX(), currentPos.getY(), angle_to_face)
            angle_turn = currentPos.angleTurnTo(pos)
            distance = currentPos.distanceTo(position)
            commands.append((toDegree(angle_turn), distance, position))
            currentPos = pp.Position(pos.getX() + distance * math.cos(angle_to_face),
                                     pos.getY() + distance * math.sin(angle_to_face), angle_to_face)
            currentPos.orientation = angle_to_face
    return commands


"""
Robot state related functions
"""


def waitForLocalization():
    global motor_pub
    pos = -150
    mc.port_looky(motor_pub, pos)
    mc.port_looky(motor_pub, pos)
    rospy.sleep(1)
    while currentState.getCurrentPos() is None and pos < 150:
        print 'move tag'
        pos += 15
        mc.port_looky(motor_pub, pos)
        mc.star_looky(motor_pub, pos)
        rospy.sleep(1)  # sleep for 10 milliseconds

    if currentState.getCurrentPos() is None:
        print "Cannot locate the tag"
        exit(-1)


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
        currentState.setAcce0X(msg.value)
    elif msg.sensorID == 19:
        currentState.setGyro1Z(msg.value)
    elif msg.sensorID == 20:
        currentState.setAcce1X(msg.value)
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
    if currentState.currentPos is None or currentState.ignore_obs:
        return
    robot_orient = currentState.getCurrentPos().getOrientation()
    true_dx = msg.y * math.cos(robot_orient) + msg.x * math.cos(robot_orient - math.pi / 2)
    true_dy = msg.y * math.sin(robot_orient) + msg.x * math.sin(robot_orient - math.pi / 2)
    rad_clear = min(msg.diameter / 2 + 0.75, math.sqrt(true_dx ** 2 + true_dy ** 2) - 0.25)
    obs = pp.Obstacle(currentState.getCurrentPos().getX() + true_dx, currentState.getCurrentPos().getY() + true_dy,
                      rad_clear)
    if obs.getCenter()[0] < 0.25 or obs.getCenter()[0] > ARENA_WIDTH - 0.25 or obs.getCenter()[1] < 2.74+0.25 or obs.getCenter()[1] > 1.9+2.74 - 0.25:
        return
    if currentState.addObstacle(msg.obsID, obs):
        print 'saw obstacle'
        mc.drive_left_motor(motor_pub, 0)
        mc.drive_right_motor(motor_pub, 0)


def updatePos(msg):
    global currentState
    pos = pp.Position(msg.x, msg.y, msg.theta)
    if not currentState.oriented:
        currentState.setCurrentPos(pos)
    currentState.foundTag()


def logData(command_type, rpm, acceX, gyro, dist, angle, currentPos):
    global logfile
    log = ''
    if command_type == 'd':
        log = 'command:drive '
    else:
        log = 'command:turn'
    log += ' rpm:' + str(rpm) + ' acceX:' + str(acceX) + ' gyro:' + str(gyro) + ' goal_dist:' + str(dist) + \
           ' goal_angle:' + str(angle) + ' x_pos:' + str(currentPos.getX()) + ' y_pos:' + str(currentPos.getY())
    logfile.write(log + '\n')


def subscribe():
    rospy.Subscriber('sensorValue', sensorValue, updateState)
    rospy.Subscriber('localization_data', Localization, updatePos)
    rospy.Subscriber('obstacleDetection', Obstacle, updateObstacle)
    rospy.Subscriber('opticalFlow', Float32, optical_disp)


def shutdownRoutine():
    global motor_pub
    mc.drive_left_motor(motor_pub, 0)
    mc.drive_right_motor(motor_pub, 0)


def optical_disp(msg):
    global currentState
    if currentState.disp_on:
        currentState.disp += msg.data


def run():
    dest = pp.Position(1.5, 4.0)
    path = create_path(currentState.currentPos, dest, ARENA_WIDTH, ARENA_HEIGHT, currentState.getObstacles().values())
    if path is None:
        currentState.ignore_obs = True
        currentState.obstacles = {}
        path = create_path(currentState.currentPos, dest, ARENA_WIDTH, ARENA_HEIGHT, currentState.getObstacles().values())
    commands = converToCommands(path)
    print str(commands)
    done = False
    while not done:
        for command in commands:
            print str(command[2])
            direction = True
            if command[0] < 0:
                direction = False
            okay = turn(math.fabs(command[0]), direction, ROBOT_SPEED_TURN)
            print 'finished turn'
            if not okay:
                break
            distance = command[1]
            next_x = currentState.getCurrentPos().getX() + math.fabs(distance) * math.cos(
                currentState.getCurrentPos().getOrientation())
            next_y = currentState.getCurrentPos().getY() + math.fabs(distance) * math.sin(
                currentState.getCurrentPos().getOrientation())
            next_pos = pp.Position(next_x, next_y)
            direction = True

            if distance < 0:
                direction = False

            okay = rpmdrive(next_pos, direction, distance, ROBOT_SPEED_DRIVE)
            if not okay:
                break

            rospy.sleep(0.01)

            if currentState.getCurrentPos().distanceTo(command[2]) > 0.2:
                break

        if currentState.getCurrentPos() == dest or currentState.getCurrentPos().distanceTo(dest) < 0.2:
            done = True
        else:
            currentState.obstacle_found = False
            print "modifying path"
            path = create_path(currentState.getCurrentPos(), dest, ARENA_WIDTH, ARENA_HEIGHT,
                               currentState.getObstacles().values())

            if path is None:
                currentState.ignore_obs = True
                currentState.obstacles = {}
                path = create_path(currentState.getCurrentPos(), dest, ARENA_WIDTH, ARENA_HEIGHT, currentState.getObstacles().values())
            commands = converToCommands(path)
            print str(commands)
        if rospy.is_shutdown():
            exit(-1)


def testMode():
    tests = [transit_test, transit_dig_test, single_run_test, full_test, rpm_drive_test]
    print "Testing mode"
    print "Enter which test you want (in number):"
    print "0. transit only"
    print "1. transit + digging"
    print "2. One single run"
    print "3. Full Competition run"
    print "4. RPM drive test"
    option = int(raw_input())
    tests[option]()
    exit(-1)


def transit_test():
    mc.drive_left_motor(motor_pub, 0)
    mc.drive_right_motor(motor_pub, 0)
    x_pos = float(raw_input("Enter x_pos of destination"))
    y_pos = float(raw_input("Enter y_pos of destination"))

    print currentState.getCurrentPos()
    raw_input('start?')

    dest = pp.Position(x_pos, y_pos)
    path = create_path(currentState.currentPos, dest, ARENA_WIDTH, ARENA_HEIGHT, currentState.getObstacles().values())

    commands = converToCommands(path)
    print str(commands)
    done = False
    while not done:
        for command in commands:
            print str(command[2])
            direction = True
            if command[0] < 0:
                direction = False
            okay = turn(math.fabs(command[0]), direction, ROBOT_SPEED_TURN)
            if not okay:
                break
            distance = command[1]
            next_x = currentState.getCurrentPos().getX() + math.fabs(distance) * math.cos(
                currentState.getCurrentPos().getOrientation())
            next_y = currentState.getCurrentPos().getY() + math.fabs(distance) * math.sin(
                currentState.getCurrentPos().getOrientation())
            next_pos = pp.Position(next_x, next_y)
            direction = True

            if distance < 0:
                direction = False

            okay = rpmdrive(next_pos, direction, distance, ROBOT_SPEED_DRIVE)
            if not okay:
                break

            rospy.sleep(0.01)

            if currentState.getCurrentPos().distanceTo(command[2]) > 0.2:
                break

        if currentState.getCurrentPos() == dest or currentState.getCurrentPos().getDistanceTo(dest) < 0.2:
            done = True
        else:
            print "modifying path"
            path = create_path(currentState.getCurrentPos(), dest, ARENA_WIDTH, ARENA_HEIGHT,
                               currentState.getObstacles().values())
            commands = converToCommands(path)
            print str(commands)
        if rospy.is_shutdown():
            exit(-1)


def transit_dig_test():
    transit_test()
    t = float(raw_input('How long do you want to dig?'))
    mc.bucket_angle_actuator(motor_pub, 60)
    rospy.sleep(5)
    mc.bucket_drive_motor(motor_pub, 150)
    rospy.sleep(1)
    mc.bucket_translation_motor(motor_pub, 780)

    rospy.sleep(t)
    mc.bucket_drive_motor(motor_pub, 0)
    mc.bucket_translation_motor(motor_pub, 100)
    rospy.sleep(2)
    mc.bucket_angle_actuator(motor_pub, 15)
    rospy.sleep(5)


def single_run_test():
    pass


def full_test():
    pass


def rpm_drive_test():
    print "How much distance you want to move?"
    distance = float(raw_input())
    direction = True
    if distance < 0:
        direction = False
    dest_x = currentState.getCurrentPos().getX() + math.fabs(distance) * math.cos(
        currentState.getCurrentPos().getOrientation())
    dest_y = currentState.getCurrentPos().getY() + math.fabs(distance) * math.sin(
        currentState.getCurrentPos().getOrientation())
    dest = pp.Position(dest_x, dest_y)
    rpmdrive(dest, direction, distance, ROBOT_SPEED_DRIVE)
    result = float(raw_input("how much did it go?"))
    rospy.loginfo("goal:" + str(distance) + " actual:" + str(result) + "\n")
    exit(0)


def main():
    rospy.init_node("automodule")
    rospy.on_shutdown(shutdownRoutine)
    subscribe()
    waitForLocalization()
    global logfile
    logfile = open(str(sys.path[1]) + '/logs/2019_competition' + time.strftime("%b-%a-%d-%H-%M-%S.txt"), "w")
    run()
    rospy.spin()


if __name__ == "__main__": main()
