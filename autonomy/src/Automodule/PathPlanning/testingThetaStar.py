from PathTesting import drawPath
from ThetaStar import thetaStar
from Obstacle import Obstacle
from PathPlanning import Position
from Vertex import Vertex
import random
neighbors = []

startpos = Position(5, 5, 0, 1)
endpos = Position(700, 700, 0, 1)
start = Vertex(0, 0, 900, neighbors, startpos)
start.setParent(start)
end = Vertex(None, 900, 0, neighbors, endpos)

#n1pos  = Position(300, 100, 0, 1)
#n2pos  = Position(700, 5, 0, 1)
#n3pos = Position(400, 650, 0, 1)
#visiblen1 = Vertex(start, 400, 800, [start], n1pos)
#visiblen2 = Vertex(start, 500, 500, [start, visiblen1, end], n2pos)
#visiblen3 = Vertex(start, 500, 300, [start, visiblen1, visiblen2, end], n3pos)
#start.addNeighbor(visiblen1)
#start.addNeighbor(visiblen2)
#start.addNeighbor(visiblen3)
#end.addNeighbor(visiblen1)
#end.addNeighbor(visiblen2)
#end.addNeighbor(visiblen3)
#end.setParent(visiblen1)
#obstacles = [Obstacle(500, 600, 50), Obstacle(500, 500, 10)]
obstacles = []
positions = []
vertices=[start]
for i in range(10):
    for j in range(10):
        if(i==7 and j==7):
            continue
        neighbor = Vertex(start, (i+j)*100, 1800-(i+j)*100, [start], Position(i*100, j*100, 0, 0))
        vertices.append(neighbor)
        positions.append(Position(i*100, j*100, 0, 0))
        start.addNeighbor(neighbor)
        end.addNeighbor(neighbor)
for i in range(3):
    r1 = random.randint(5, 1000)
    r2 = random.randint(5, 1000)
    r3 = random.randint(5, 50)
    obstacles.append(Obstacle(r1, r2, r3))
obstacles.append(Obstacle(400, 400, 5))
path = thetaStar(start, end, obstacles, 1)
path.printPath()
drawPath(path, obstacles, positions)