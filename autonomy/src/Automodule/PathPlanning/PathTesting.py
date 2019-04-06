
from graphics import *
import collections
from collections import deque

def drawPath(path, obstacles, positions):
    win = GraphWin("Path", 1000, 800)
    lastX = 0
    lastY = 0
    lines = deque()
    points = deque()
    for position in positions:
        c = Circle(Point(position.getX_pos(), position.getY_pos()), 5)
        c.setFill("green")
        c.setOutline("green")
        points.append(c)
    for position in path.path:
        if lastX != 0 or lastY != 0:
            l = Line(Point(lastX, lastY), Point(position.getX_pos(), position.getY_pos()))
            lines.append(l)
        c = Circle(Point(position.getX_pos(), position.getY_pos()), 5)
        c.setFill("blue")
        c.setOutline("blue")
        points.append(c)
        lastX = position.getX_pos()
        lastY = position.getY_pos()
    for obstacle in obstacles:
        c = Circle(Point(obstacle.center_x, obstacle.center_y), obstacle.getRadius())
        c.setFill("red")
        c.setOutline("red")
        points.append(c)
    for x in lines:
        x.draw(win)
    for x in points:
        x.draw(win)
    win.getMouse()
    win.close()