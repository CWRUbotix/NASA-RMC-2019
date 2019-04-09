#!/usr/bin/env python

from collections import deque
import math
from PathPlanning import Grid, Path

open = deque()
closed = deque()


def aStar(start, end):
    openList = [start]
    start.setStartDistance(0)
    start.setEndDistance(start.getPosition().distanceTo(end.getPosition()))
    closedList = []
    while openList.__len__() > 0:
        bestDist = openList[0].getStartDistance() + openList[0].getEndDistance()
        bestIndex = 0
        # find next vertex to try
        for i in range (1, openList.__len__()):
            if (openList[i].getStartDistance() + openList[i].getEndDistance()) < bestDist:
                bestDist = openList[i].getStartDistance() + openList[i].getEndDistance()
                bestIndex = i
        if openList[bestIndex] == end:
            # reached end, need to create path then exit
            path = deque()
            node = openList[bestIndex]
            while node is not None:
                path.appendleft(node.getPosition)
                node = node.getParent()
            return
        # add neighbors of best looking vertex if they aren't in the closed list
        for node in openList[bestIndex].getVisibleNeighbors:
            # iterate through closed list to check if node has already been considered
            inClosed = False
            for nodeTwo in closedList:
                if node == nodeTwo:
                    inClosed = True
            tempScore = openList[bestIndex].getStartDistance() + node.getPosition().distanceTo()(openList[bestIndex].getPosition())
            tempScore += node.getPosition().distanceTo(end.getPosition())
            inOpen = False
            # iterate through open list to check if node can be reached in a better way
            for nodeTwo in openList:
                if node == nodeTwo:
                    inOpen = True
                    if tempScore < nodeTwo.getStartDistance() + nodeTwo.getEndDistance():
                        nodeTwo.setParent(openList[bestIndex])
                        node.setStartDistance(node.getParent().getStartDistance() + node.getPosition().distanceTo(
                            node.getParent().getPosition()))
                        node.setEndDistance(node.getPosition().distanceTo(end.getPosition()))
            # if node is new, add it to the open set
            if not inClosed and not inOpen:
                node.setParent(openList[bestIndex])
                node.setStartDistance(node.getParent().getStartDistance() + node.getPosition().distanceTo(node.getParent().getPosition()))
                node.setEndDistance(node.getPosition().distanceTo(end.getPosition()))
                openList.append(node)
        closedList.append(openList[bestIndex])
        openList.remove(openList[bestIndex])

def thetaStar(start, goal, obstacles, unitScale):
    start.setStartDistance(0)
    start.setParent(start)
    open.insert(int(start.getStartDistance() + start.getEndDistance()), start)
    while open.__len__() > 0:
        s = open.pop()
        if s == goal:
            return reconstruct_path(s)
        closed.append(s)
        for neighbor in s.getVisibleNeighbors():
            if neighbor not in closed:
                if neighbor not in open:
                    neighbor.setStartDistance(math.inf)
                    neighbor.setParent(None)
            update_vertex(s, neighbor, obstacles, unitScale)
    return None

def update_vertex(vertex, neighbor, obstacles, unitScale):
    if line_of_sight(vertex.getParent(), neighbor, obstacles, unitScale):
        if vertex.getParent().getStartDistance() + vertex.getParent().getPosition().distanceTo(neighbor.getPosition()) < neighbor.getStartDistance():
            neighbor.setStartDistance(vertex.getParent().getStartDistance() + vertex.getParent().getPosition().distanceTo(neighbor.getPosition()))
            neighbor.setParent(vertex.getParent())
            if neighbor in open:
                open.remove(neighbor)
            open.insert(int(neighbor.getStartDistance() + neighbor.getEndDistance()), neighbor)
    else:
        if vertex.getStartDistance() + vertex.getPosition().distanceTo(neighbor.getPosition()) < neighbor.getStartDistance():
            neighbor.setStartDistance(vertex.getStartDistance() + vertex.getPosition().distanceTo(neighbor.getPosition()))
            neighbor.setParent(vertex)
            if neighbor in open:
                open.remove(neighbor)
            open.insert(int(neighbor.getStartDistance() + neighbor.getEndDistance()), neighbor)

total_path = []
def reconstruct_path(vertex):

    if vertex.getParent() != vertex:
        total_path.insert(0, vertex.getPosition())
        parent = vertex.getParent()
        if(vertex.getParent().getParent() == vertex.getParent()):
            total_path.insert(0, parent.getPosition())
        return reconstruct_path(parent)
    else:
        path = Path(total_path)
        return path

def line_of_sight(vertex, vertexTwo, obstacles, unitScale):
    x0 = vertex.getPosition().getX_pos()
    x1 = vertexTwo.getPosition().getX_pos()
    y0 = vertex.getPosition().getY_pos()
    y1 = vertexTwo.getPosition().getY_pos()
    dy = y1 - y0
    dx = x1 - x0
    f = 0
    sx = 0
    sy = 0
    if dy < 0:
        dy = -dy
        sy = -1
    else:
        sy = 1
    if dx < 0:
        dx = -dx
        sx = -1
    else:
        sx = 1
    if dx >= dy:
        while x0 != x1:
            f += dy
            if f >= dx:
                if Grid.gridBlocked(x0+((sx-1)/2), y0+((sy-1)/2), obstacles, unitScale):
                    return False
                y0 += sy
                f -= dx
            if f != 0 and Grid.gridBlocked(x0+((sx-1)/2), y0+((sy-1)/2), obstacles, unitScale):
                return False
            if dy == 0 and Grid.gridBlocked(x0+((sx-1)/2), y0, obstacles, unitScale) & Grid.gridBlocked(x0+((sx-1)/2), y0-1, obstacles, unitScale):
                return False
            x0 += sx
    else:
        while y0 != y1:
            f += dx
            if f >= dy:
                if Grid.gridBlocked(x0+((sx-1)/2), y0+((sy-1)/2), obstacles, unitScale):
                    return False
                x0 += sx
                f -= dy
            if f != 0 and Grid.gridBlocked(x0+((sx-1)/2), y0+((sy-1)/2), obstacles, unitScale):
                return False
            if dx == 0 and Grid.gridBlocked(x0, y0+((sy-1)/2), obstacles, unitScale) & Grid.gridBlocked(x0-1, y0+((sy-1)/2), obstacles, unitScale):
                return False
            y0 += sy
    return True
