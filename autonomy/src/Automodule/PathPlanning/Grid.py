class Grid:

    def __init__(self, vertices, obstacles):
        self.vertices = vertices
        self.obstacles = obstacles

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
    def gridBlocked(Xpos, Ypos, obstacles):
        xBlocked = False
        yBlocked = False
        for obstacle in obstacles:
            if obstacle.center_x >= Xpos:
                xBlocked = (obstacle.getCenter()[0] - obstacle.getRadius()) < Xpos + 1
            else:
                xBlocked = (obstacle.getCenter()[0] + obstacle.getRadius()) > Xpos
            if obstacle.center_y >= Ypos:
                yBlocked = (obstacle.getCenter()[1] - obstacle.getRadius()) < Ypos + 1
            else:
                yBlocked = (obstacle.getCenter()[1] + obstacle.getRadius()) > Ypos
        return xBlocked & yBlocked

