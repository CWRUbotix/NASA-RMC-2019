#!/usr/bin/env python

import heapq as h
from PathPlanning import Grid, Position, Vertex, Path
import math

def create_path(start, end, areana_width, arena_height, obstacles):
    grid = Grid(start, end, areana_width, arena_height)
    for obs in obstacles:
        grid.addObstacle(obs)

    for i in range(grid.row_size):
        for j in range(grid.col_size):
            v = grid.getVertex(i, j)
            neighbors = grid.getNeighbors(i, j)
            for neighbor in neighbors:
                v.addNeighbor(neighbor)

    grid_start_coord = grid.getGridCoordinates(start.getX(), start.getY())
    grid_end_coord = grid.getGridCoordinates(end.getX(), end.getY())

    return thetaStar(grid_start_coord, grid_end_coord, grid)

def thetaStar(start, end, grid):
    startVertex = grid.getVertex(start[0], start[1])
    startVertex.setDistance(0)
    endVertex = grid.getVertex(end[0], end[1])
    updateHeuristic(endVertex, grid)
    openList = []
    closedList = []

    h.heappush(openList, startVertex)
    while not len(openList) != 0:
        currentVertex = h.heappop(openList)

        if currentVertex == endVertex:
            return postProcess(reconstructPath(currentVertex), grid)

        closedList.append(currentVertex)

        for neighbor in currentVertex.getNeighbors():
            if neighbor not in closedList:
                temp = currentVertex.getDistance() + currentVertex.getDistanceTo(neighbor)
                if temp < neighbor.getDistance():
                    neighbor.setDistance(currentVertex.getDistance() + currentVertex.getDistanceTo(neighbor))
                    neighbor.setParent(currentVertex)
                    if neighbor in openList:
                        h.heapify(openList)
                if neighbor not in openList:
                    h.heappush(openList, neighbor)

    return None

def reconstructPath(v, acc=None):
    if acc is None:
        acc = []
    acc.insert(0, v)
    if v.getParent() is None:
        return Path(acc)
    else:
        return reconstructPath(v.getParent(), acc)

def updateHeuristic(end, grid):
    for i in range(grid.row_size):
        for j in range(grid.col_size):
            v = grid.getVertex(i, j)
            v.setHeuristic(v.distanceTo(end))

def postProcess(path, grid):
    current = 0
    done = False
    while not done:
        if len(path.path) >= current + 3:
            if not checkBlocked(path.path[current], path.path[current + 2], grid):
                path.path[current + 2].setParent(path.path[current])
                path.delete(path.path[current + 1])
            else:
                current += 1
        else:
            done = True

    return path

def checkBlocked(p1, p2, grid):
    
    pass

