from PathTesting import drawPath
from ThetaStar import thetaStar
from Obstacle import Obstacle
from PathPlanning import Position
from Vertex import Vertex
neighbors = []

startpos = Position(0, 0, 0, 1)
endpos = Position(800, 800, 0, 1)
start = Vertex(0, 0, 800, neighbors, startpos)
start.setParent(start)
end = Vertex(start, 800, 0, neighbors, endpos)
n1pos  = Position(300, 100, 0, 1)
n2pos  = Position(800, 0, 0, 1)
visiblen1 = Vertex(start, 0, 800, [start], n1pos)
visiblen2 = Vertex(start, 0, 800, [start, visiblen1, end], n2pos)
start.addNeighbor(visiblen1)
start.addNeighbor(visiblen2)
end.addNeighbor(visiblen1)
end.addNeighbor(visiblen2)
obstacles = [Obstacle(500, 600, 100)]
path = thetaStar(start, end, obstacles, 1)
path.printPath()
drawPath(path)