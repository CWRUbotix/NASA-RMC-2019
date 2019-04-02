from PathTesting import drawPath
from ThetaStar import thetaStar
from Obstacle import Obstacle
from PathPlanning import Position
from Vertex import Vertex
neighbors = []

startpos = Position(5, 5, 0, 1)
endpos = Position(700, 700, 0, 1)
start = Vertex(0, 0, 900, neighbors, startpos)
start.setParent(start)
end = Vertex(start, 900, 0, neighbors, endpos)
n1pos  = Position(300, 100, 0, 1)
n2pos  = Position(700, 5, 0, 1)
visiblen1 = Vertex(start, 400, 800, [start], n1pos)
visiblen2 = Vertex(start, 600, 800, [start, visiblen1, end], n2pos)
start.addNeighbor(visiblen1)
start.addNeighbor(visiblen2)
end.addNeighbor(visiblen1)
end.addNeighbor(visiblen2)
end.setParent(visiblen1)
obstacles = [Obstacle(500, 600, 50), Obstacle(500, 500, 10)]
path = thetaStar(start, end, obstacles, 1)
path.printPath()
drawPath(path, obstacles)