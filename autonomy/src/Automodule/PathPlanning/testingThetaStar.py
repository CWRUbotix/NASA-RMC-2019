from PathTesting import drawPath
from ThetaStar import thetaStar
from Obstacle import Obstacle
from PathPlanning import Position
from Vertex import Vertex
import random
neighbors = []

startpos = Position(5, 5, 0, 1)
endpos = Position(700, 700, 0, 1)
start = Vertex(0, 0, 1272, neighbors, startpos)
start.setParent(start)
end = Vertex(None, 1272, 0, neighbors, endpos)


obstacles = []
positions = []
vertices=[[0 for x in range(10)] for y in range(10)]
for i in range(10):
    for j in range(10):

        point = Vertex(None, (i+j)*100, (((9-i)**2 + (9-j)**2)**1/2)*100, [], Position(i*100, j*100, 0, 0))
        if (i == 7 and j == 7):
            vertices[i][j] = end
        elif(i==0 & j==0):
            vertices[i][j] = start
        else:
            vertices[i][j] = point
for i in range(10):
    for j in range(10):
        if (i + 1 <= 9):
            vertices[i][j].addNeighbor(vertices[i + 1][j])
            if (j + 1 <= 9):
                #vertices[i][j].addNeighbor(vertices[i + 1][j + 1])
                vertices[i][j].addNeighbor(vertices[i][j + 1])
            if (j - 1 >= 0):
                #vertices[i][j].addNeighbor(vertices[i + 1][j - 1])
                vertices[i][j].addNeighbor(vertices[i][j - 1])
        if (i - 1 >= 0):
            vertices[i][j].addNeighbor(vertices[i - 1][j])
            if (j + 1 <= 9):
                #vertices[i][j].addNeighbor(vertices[i - 1][j + 1])
                None
            if (j - 1 >= 0):
                #vertices[i][j].addNeighbor(vertices[i - 1][j - 1])
                None

        positions.append(Position(i * 100, j * 100, 0, 0))

vertices[0][0].setParent(start)
for i in range(15):
    r1 = random.randint(150, 1000)
    r2 = random.randint(150, 1000)
    r3 = random.randint(5, 50)
    obstacles.append(Obstacle(r1, r2, r3))
obstacles.append(Obstacle(400, 400, 5))
path = thetaStar(start, end, obstacles, 1)
path.printPath()
drawPath(path, obstacles, positions)