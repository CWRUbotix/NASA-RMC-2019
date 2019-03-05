#!/usr/bin/env python

class Grid:

    def __init__(self, vertices):
        self.vertices = vertices

    def getVertices(self):
        return self.vertices

    def removeVertex(self, vertex):
        self.vertices.remove(vertex)

    def addVertex(self, vertex):
        self.vertices.append(vertex)
