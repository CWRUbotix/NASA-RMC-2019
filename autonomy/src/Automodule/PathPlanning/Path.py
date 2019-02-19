import collections
from collections import deque

class Path(collections.Sequence):
    def __init__(self, positions):
        self.path = deque(positions)

    def insert(self, newPositions):
        self.path.append(newPositions)

    def get_Position(self):
        return self.path.pop()

    def __len__(self):
        return len(self.path)

    def delete(self, position):
        return self.path.remove(position)

    def printPath(self):
        for position in self.path:
            print ("X: %s" %(position.getX_pos()))
            print ("Y: %s" %(position.getY_pos()))
            print ("Orientation: %s\n" %(position.getOrientation()))