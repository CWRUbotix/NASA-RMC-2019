#!/usr/bin/env python

import math
import collections
from collections import deque

#Global Variables
ERROR_BOUND = 0.05
CLEARANCE = 0.75
GRID_SIZE = 0.25

class Position:
    def __init__(self, x_pos, y_pos, orientation=0.0):
        self.x_pos = x_pos
        self.y_pos = y_pos
        self.orientation = orientation

    def getX(self):
        return self.x_pos

    def getY(self):
        return self.y_pos

    def getOrientation(self):
        return self.orientation

    def setOrientation(self, o):
        self.orientation = o

    def __str__(self):
        return "x: " + str(self.x_pos) + " y: " + str(self.y_pos) + " angle: " + str(self.orientation) + " radians."

    def __eq__(self, p):
        if self.distanceTo(p) < ERROR_BOUND:
            return True
        return False

    def distanceTo(self, p):
        x = abs(p.getX() - self.x_pos)
        y = abs(p.getY() - self.y_pos)
        return (x ** 2 + y ** 2) ** .5

    def angleTurnTo(self, p):
        if p.getOrientation() - self.orientation > 0:
            return (p.getOrientation() - self.orientation) % (2 * math.pi)
        else:
            return ((p.getOrientation() - self.orientation) % (2 * math.pi)) - 2 * math.pi

    def angleToFace(self, p):
        return (math.atan2(p.getY() - self.getY(), p.getX() - self.getY()) + 2 * math.pi) % (2 * math.pi)

class Grid:

    def __init__(self, p1, p2, width, height, idealUnit=GRID_SIZE):
        col_size = math.floor(math.fabs((p2.getX() - p1.getX()) / idealUnit))
        row_size = math.floor(math.fabs((p2.getY() - p1.getY()) / idealUnit))
        true_unit_width = math.fabs((p1.getX() - p2.getX()) / col_size)
        true_unit_height = math.fabs((p1.getX() - p2.getX()) / row_size)

        self.p1 = Position(CLEARANCE + ((p1.getX() - CLEARANCE) % true_unit_width), CLEARANCE + ((p1.getY() - CLEARANCE) % true_unit_height))
        self.p2 = Position(width - CLEARANCE - ((width - CLEARANCE - p2.getX()) % true_unit_width), height - CLEARANCE - ((height - CLEARANCE - p2.getY()) % true_unit_height))
        self.width = self.p2.getX() - self.p1.getX()
        self.height = self.p2.getY() - self.p2.getY()
        self.col_size = math.floor(width / true_unit_width)
        self.row_size = math.floor(height / true_unit_height)
        self.unit_width = true_unit_width
        self.unit_height = true_unit_height
        self.vertices = [[]]
        self.cells = []

        for i in range(self.row_size - 1):
            self.vertices.append([])
            self.cells.append([])
            for j in range(self.col_size - 1):
                self.cells[i].append(False)

        for i in range(self.row_size):
            for j in range(self.col_size):
                self.vertices[i].append(Vertex(self.p1.getX() + i * self.unit_width, self.p1.getY() + j * self.unit_height))

    def getVertex(self, row_index, col_index):
        return self.vertices[row_index][col_index]

    def getNeighbors(self, row_index, col_index):
        neighbors = []
        x_coords = [col_index - 1, col_index, col_index + 1]
        y_coords = [row_index - 1, row_index, row_index + 1]

        for i in range(3):
            for j in range(3):
                if not (i == row_index and j == col_index) \
                          and not (x_coords[i] < 0 or y_coords[i] < 0) \
                          and not (x_coords [i] > self.width or y_coords[j] > self.height) \
                          and not self.blocked(x_coords[i], y_coords[j]):
                    neighbors.append(self.getVertex(x_coords[i], y_coords[j]))

        return neighbors

    def blocked(self, row_index, col_index):
        return self.cells[row_index][col_index]

    def addObstacle(self, obs):
        o1 = [max(0, math.floor((obs.getCenter()[0] - obs.getRadius() - CLEARANCE) / self.unit_width)),
              max(0, math.floor((obs.getCenter()[1] - obs.getRadius() - CLEARANCE) / self.unit_height))]
        o2 = [min(self.col_size - 2, math.ceil((obs.getCenter()[0] + obs.getRadius() + CLEARANCE) / self.unit_width)),
              min(self.row_size - 2, math.ceil((obs.getCenter()[1] + obs.getRadius() + CLEARANCE) / self.unit_height))]
        for i in range(o1[1], o2[1] + 1):
            for j in range(o1[0], o2[0] + 1):
                self.cells[i][j] = True

    def getGridCoordinates(self, x_pos, y_pos):
        col_index = min(self.col_size - 1, max(round((x_pos - self.p1.getX()) / self.unit_width), 0))
        row_index = min(self.row_size - 1, max(round((y_pos - self.p1.getY()) / self.unit_height), 0))
        return row_index, col_index

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

class Vertex(Position):
    def __init__(self, x_pos, y_pos):
        super(Vertex, self).__init__(x_pos, y_pos, 0)
        self.parent = None
        self.dist = math.inf
        self.heuristic = math.inf
        self.visibleNeighbors = []

    def __lt__(self, other):
        return self.dist + self.heuristic < other.getDistance() + other.getHeuristic()

    def __gt__(self, other):
        return self.dist + self.heuristic > other.getDistance() + other.getHeuristic()

    def __le__(self, other):
        return self.dist + self.heuristic <= other.getDistance() + other.getHeuristic()

    def __ge__(self, other):
        return self.dist + self.heuristic >= other.getDistance() + other.getHeuristic()

    def getParent(self):
        return self.parent

    def setParent(self, parent):
        self.parent = parent

    def getVisibleNeighbors(self):
        return self.visibleNeighbors

    def removeNeighbor(self, vertex):
        self.visibleNeighbors.remove(vertex)

    def addNeighbor(self, vertex):
        self.visibleNeighbors.append(vertex)

    def getDistance(self):
        return self.dist

    def setDistance(self, dist, reset=False):
        if reset:
            self.dist = math.inf
        else:
            self.dist = dist

    def getHeuristic(self):
        return self.heuristic

    def setHeuristic(self, dest, reset=False):
        if reset:
            self.heuristic = math.inf
        else:
            self.heuristic = self.distanceTo(dest)
