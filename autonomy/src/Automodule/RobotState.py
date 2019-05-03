import math

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

    def setBCArmUpperLimit(self, bcarm_upper_limit_switch):
	self.bcarm_upper_limit_switch = bcarm_upper_limit_switch

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

