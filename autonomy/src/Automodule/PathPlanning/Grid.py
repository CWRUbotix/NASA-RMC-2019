#!/usr/bin/env python

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
                xBlocked = (obstacle.getCenter()[0] - obstacle.getRadius()) < Xpos + unitScale
            else:
                xBlocked = (obstacle.getCenter()[0] + obstacle.getRadius()) > Xpos
            if obstacle.center_y >= Ypos:
                yBlocked = (obstacle.getCenter()[1] - obstacle.getRadius()) < Ypos + unitScale
            else:
                yBlocked = (obstacle.getCenter()[1] + obstacle.getRadius()) > Ypos
        return (xBlocked and yBlocked)

