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

    #returns true if the grid space with lower left vertex at (Xpos, Ypos) is obstructed by an obstacle
    def gridBlocked(self, Xpos, Ypos):
        x = False
        y = False
        for obstacle in self.obstacles:
            if obstacle.center_x >= Xpos:
                x = (obstacle.getCenter()(1) - obstacle.getRadius()) < Xpos + 1
            else:
                x = (obstacle.getCenter()(1) + obstacle.getRadius()) > Xpos
            if obstacle.center_y >= Ypos:
                y = (obstacle.getCenter()(1) - obstacle.getRadius()) < Ypos + 1
            else:
                y = (obstacle.getCenter()(1) + obstacle.getRadius()) > Ypos
        return (x & y)

