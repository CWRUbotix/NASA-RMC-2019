import collections

class Path(collections.Sequence):
    def __init__(positions):
        self.path = deque(positions)

    def insert(self, newPositions):
        self.path.append(newPositions)

    def get_Position(self):
        return self.path.pop()

    def __len__(self):
        return len(self.path)

    def delete(self, position):
        return self.path.remove(position)

    