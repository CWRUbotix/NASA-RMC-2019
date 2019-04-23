
from graphics import *
import collections
from collections import deque

def drawPath(path, obstacles, positions):
    win = GraphWin("Path", 800, 1600)
    lastX = 0
    lastY = 0
    lines = deque()
    points = deque()
    for position in positions:
        c = Circle(Point(position.getX()*150, position.getY()*150), 5)
        c.setFill("green")
        c.setOutline("green")
        points.append(c)
    for i in range(len(path)):
        p = path.getPosition()
        print str(p)
        if lastX != 0 or lastY != 0:
            l = Line(Point(lastX, lastY), Point(p.getX()*150, p.getY()*150))
            lines.append(l)
        c = Circle(Point(p.getX()*150, p.getY()*150), 7)
        c.setFill("blue")
        c.setOutline("blue")
        points.append(c)
        lastX = p.getX() * 150
        lastY = p.getY() * 150
    for obstacle in obstacles:
        c = Circle(Point(obstacle.center_x*150, obstacle.center_y*150), obstacle.getRadius()*150)
        c.setFill("red")
        c.setOutline("red")
        points.append(c)
    for x in lines:
        x.draw(win)
    for x in points:
        x.draw(win)
    win.getMouse()
    win.close()