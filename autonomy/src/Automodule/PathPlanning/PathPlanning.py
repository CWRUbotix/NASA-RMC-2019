class Position:
    def __init__(self, X_pos, Y_pos, Orientation, Error_bound):
        self.X_pos = X_pos
        self.Y_pos = Y_pos
        self.Orientation = Orientation
        self.Error_bound = Error_bound

    def getX_pos(self):
        return self.X_pos

    def getY_pos(self):
        return self.Y_pos

    def getOrientation(self):
        return self.Orientation

    def setOrientation(self, o):
        Orientation = o

    def __str__(self):
        return "x: " + self.X_pos + " y: " + self.Y_pos + " angle: " + self.Orientation + " radians."

    def __eq__(self, p):
        if abs(p.getX_pos() - self.X_pos) <= self.Error_bound:
            if abs(p.getY_pos() - self.Y_pos) <= self.Error_bound:
                if abs(p.getOrientation() - self.Orientation) <= self.Error_bound:
                    return True
        return False

    def distanceTo(self, p):
        x = abs(p.getX_pos - self.X_pos)
        y = abs(p.getY_pos - self.Y_pos)
        return (x ** 2 + y ** 2) ** (.5)

    def angleToFace(self, p):
        return p.getOrientation - self.Orientation