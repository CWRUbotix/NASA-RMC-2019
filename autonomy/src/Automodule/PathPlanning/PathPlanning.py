#!/usr/bin/env python

import math
import collections
from collections import deque

#Global Variables
ERROR_BOUND = 0.05
CLEARANCE = 0.75
GRID_SIZE = 0.1

class Position(object):
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
        if math.fabs(self.getX() - p.getX()) < ERROR_BOUND and math.fabs(self.getY() - p.getY()) < ERROR_BOUND:
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
        return math.atan2(p.getY() - self.getY(), p.getX() - self.getY()) % (2 * math.pi)


class Grid(object):

    def __init__(self, p1, p2, width, height, idealUnit=GRID_SIZE):
        if math.fabs((p2.getX() - p1.getX())) < idealUnit:
            true_unit_width = math.fabs((p2.getX() - p1.getX()))
        else:
            col_size = math.floor(math.fabs((p2.getX() - p1.getX()) / idealUnit))
            true_unit_width = math.fabs((p1.getX() - p2.getX()) / col_size)

        if math.fabs((p2.getY() - p1.getY())) < idealUnit:
            true_unit_height = math.fabs((p2.getY() - p1.getY()))
        else:
            row_size = math.floor(math.fabs((p2.getY() - p1.getY()) / idealUnit))
            true_unit_height = math.fabs((p1.getY() - p2.getY()) / row_size)

        self.p1 = Position(min(p1.getX(), p2.getX()), min(p1.getY(), p2.getY()))
        self.p2 = Position(max(p1.getX(), p2.getX()), max(p1.getY(), p2.getY()))
        self.p1 = Position(self.p1.getX() % true_unit_width, self.p2.getY() % true_unit_height)
        self.p2 = Position(width - (width - self.p2.getX()) % true_unit_width, height - ((height - self.p2.getY()) % true_unit_height))
        self.width = width
        self.height = width
        self.col_size = int(math.floor(width / true_unit_width))
        self.row_size = int(math.floor(height / true_unit_height))
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
                self.vertices[i].append(Vertex(self.p2.getX() - j * self.unit_width, self.p1.getY() + i * self.unit_height))

    def getVertex(self, row_index, col_index):
        return self.vertices[int(row_index)][int(col_index)]

    def getNeighbors(self, row_index, col_index):
        neighbors = []
        x_coords = [col_index - 1, col_index, col_index + 1]
        y_coords = [row_index - 1, row_index, row_index + 1]

        left_top = x_coords[0] >= 0 and y_coords[0] >= 0 and not self.blocked(y_coords[0], x_coords[0])
        left_bot = x_coords[0] >= 0 and y_coords[1] < self.row_size - 1 and not self.blocked(y_coords[1], x_coords[0])
        right_top = x_coords[1] < self.col_size - 1 and y_coords[0] >= 0 and not self.blocked(y_coords[0], x_coords[1])
        right_bot = x_coords[1] < self.col_size - 1 and y_coords[1] < self.row_size - 1 and not self.blocked(y_coords[1], x_coords[1])

        if left_top:
            neighbors.append(self.getVertex(y_coords[0], x_coords[0]))
        if left_top and left_bot:
            neighbors.append(self.getVertex(y_coords[1], x_coords[0]))
        if left_bot:
            neighbors.append(self.getVertex(y_coords[2], x_coords[0]))
        if left_bot and right_bot:
            neighbors.append(self.getVertex(y_coords[2], x_coords[1]))
        if right_bot:
            neighbors.append(self.getVertex(y_coords[2], x_coords[2]))
        if right_bot and right_top:
            neighbors.append(self.getVertex(y_coords[1], x_coords[2]))
        if right_top:
            neighbors.append(self.getVertex(y_coords[0], x_coords[2]))
        if left_top and right_top:
            neighbors.append(self.getVertex(y_coords[0], x_coords[1]))

        return neighbors

    def blocked(self, row_index, col_index):
        if row_index < 0 or row_index >= self.row_size - 1:
            return True
        if col_index < 0 or col_index >= self.col_size - 1:
            return True

        return self.cells[int(row_index)][int(col_index)]

    def addObstacle(self, obs):
        o1 = self.getGridCoordinates(obs.getCenter()[0] + obs.getRadius() + CLEARANCE, obs.getCenter()[1] - obs.getRadius() - CLEARANCE)
        o2 = self.getGridCoordinates(obs.getCenter()[0] - obs.getRadius() - CLEARANCE, obs.getCenter()[1] + obs.getRadius() + CLEARANCE)
        print str(o1)
        print str(o2)
        for i in range(int(o1[0]), int(o2[0])):
            for j in range(int(o1[1]), int(o2[1])):
                self.cells[i][j] = True

    def getGridCoordinates(self, x_pos, y_pos):
        col_index = min(self.col_size - 1, max(round((self.p2.getX() - x_pos) / self.unit_width), 0))
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

    def getPosition(self):
        return self.path.popleft()

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
        self.dist = float('inf')
        self.heuristic = float('inf')
        self.visibleNeighbors = []

    def __lt__(self, other):
        return self.dist + self.heuristic < other.getDistance() + other.getHeuristic()

    def __gt__(self, other):
        return self.dist + self.heuristic > other.getDistance() + other.getHeuristic()

    def __le__(self, other):
        return self.dist + self.heuristic <= other.getDistance() + other.getHeuristic()

    def __ge__(self, other):
        return self.dist + self.heuristic >= other.getDistance() + other.getHeuristic()

    def __eq__(self, other):
        return self.getX() == other.getX() and self.getY() == other.getY()

    def __str__(self):
        return 'x: ' + '%.4f' % self.getX() + ' y :' + '%.4f' % self.getY()

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
            self.dist = float('inf')
        else:
            self.dist = dist

    def getHeuristic(self):
        return self.heuristic

    def setHeuristic(self, dest, reset=False):
        if reset:
            self.heuristic = math.inf
        else:
            self.heuristic = dest
