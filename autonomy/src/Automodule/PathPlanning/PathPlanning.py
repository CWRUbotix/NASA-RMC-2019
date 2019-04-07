#!/usr/bin/env python

import math
import collections
from collections import deque

#Global Variables
Error_bound = 0.10


class Position:
    def __init__(self, X_pos, Y_pos, Orientation):
        self.X_pos = X_pos
        self.Y_pos = Y_pos
        self.Orientation = Orientation

    def getX_pos(self):
        return self.X_pos

    def getY_pos(self):
        return self.Y_pos

    def getOrientation(self):
        return self.Orientation

    def setOrientation(self, o):
        self.Orientation = o

    def __str__(self):
        return "x: " + str(self.X_pos) + " y: " + str(self.Y_pos) + " angle: " + str(self.Orientation) + " radians."

    def __eq__(self, p):
        if self.distanceTo(p) < Error_bound:
            return True
        return False

    def distanceTo(self, p):
        x = abs(p.getX_pos() - self.X_pos)
        y = abs(p.getY_pos() - self.Y_pos)
        return (x ** 2 + y ** 2) ** .5

    def angleToFace(self, p):
        return p.getOrientation() - self.Orientation

class Grid:

    def __init__(self, vertices, obstacles, unitScale):
        self.vertices = vertices
        self.obstacles = obstacles
        self.unitScale = unitScale

    def getVertices(self):
        return self.vertices

    def removeVertex(self, vertex):
        self.vertices.remove(vertex)

    def addVertex(self, vertex):
        self.vertices.append(vertex)

    def addObstacle(self, obstacle):
        self.obstacles.append(obstacle)

    # returns true if the grid space with lower left vertex at (Xpos, Ypos) is obstructed by an obstacle
    @staticmethod
    def gridBlocked(Xpos, Ypos, obstacles, unitScale):
        xBlocked = False
        yBlocked = False
        for obstacle in obstacles:
            if obstacle.center_x >= Xpos:
                xBlocked = (obstacle.getCenter()[0] - obstacle.getRadius()) < (Xpos + unitScale)
            else:
                xBlocked = (obstacle.getCenter()[0] + obstacle.getRadius()) > Xpos
            if obstacle.center_y >= Ypos:
                yBlocked = (obstacle.getCenter()[1] - obstacle.getRadius()) < (Ypos + unitScale)
            else:
                yBlocked = (obstacle.getCenter()[1] + obstacle.getRadius()) > (Ypos)
        return (xBlocked and yBlocked)

class Obstacle:
    def __init__(self, center_x, center_y, radius):
        self.center_x = center_x
        self.center_y = center_y
        self.radius = radius

    def getCenter(self):
        return [self.center_x, self.center_y]

    def getRadius(self):
        return self.radius

    def setRadius(self, radius):
        self.radius = radius

    def setCenter(self, center_x, center_y):
        self.center_x = center_x
        self.center_y = center_y

    def mergeIfEqual(self, other, maximum_overlap):
        distance_x = self.center_x - other.center_x
        distance_y = self.center_y - other.center_y
        distance = math.sqrt(math.pow(distance_x, 2) + math.pow(distance_y, 2))
        sum_radius = self.radius + other.radius
        if (sum_radius - distance) >= maximum_overlap * distance:
            self.center_x = (self.center_x + other.center_x) / 2
            self.center_y = (self.center_y + other.center_y) / 2
            self.radius = distance / 2

#acts as a sequence of instances of the Position class
class Path(collections.Sequence):
    def __init__(self, positions):
        self.path = deque(positions)

    def insert(self, newPositions):
        self.path.append(newPositions)

    def get_Position(self):
        return self.path.pop()

    def __getitem__(self, item):
        return item

    def __len__(self):
        return len(self.path)

    def delete(self, position):
        return self.path.remove(position)

    def printPath(self):
        for position in self.path:
            print ("X: %s" %(position.getX_pos()))
            print ("Y: %s" %(position.getY_pos()))
            print ("Orientation: %s\n" %(position.getOrientation()))

class Vertex:
    def __init__(self, parent, startDistance, endDistance, visibleNeighbors, position):
        self.parent = parent
        self.startDistance = startDistance
        self.endDistance = endDistance
        self.visibleNeighbors = visibleNeighbors
        self.position = position

    def getParent(self):
        return self.parent

    def setParent(self, parent):
        self.parent = parent

    def getStartDistance(self):
        return self.startDistance

    def setStartDistance(self, startDistance):
        self.startDistance = startDistance

    def getEndDistance(self):
        return self.endDistance

    def setEndDistance(self, endDistance):
        self.endDistance = endDistance

    def getVisibleNeighbors(self):
        return self.visibleNeighbors

    def removeNeighbor(self, vertex):
        self.visibleNeighbors.remove(vertex)

    def addNeighbor(self, vertex):
        self.visibleNeighbors.append(vertex)

    def __eq__(self, vertex):
        return self.position == vertex.position

    def getPosition(self):
        return self.position
