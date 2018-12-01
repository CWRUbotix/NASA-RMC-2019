from collections import deque

def aStar(start, end):
    openList = [start]
    start.setStartDistance(0)
    start.setEndDistance(start.getPosition().distanceTo(end.getPosition()))
    closedList = []
    while openList.__len__() > 0:
        bestDist = openList[0].getStartDistance() + openList[0].getEndDistance()
        bestIndex = 0
        # find next vertex to try
        for i in range (1, openList.__len__()):
            if (openList[i].getStartDistance() + openList[i].getEndDistance()) < bestDist:
                bestDist = openList[i].getStartDistance() + openList[i].getEndDistance()
                bestIndex = i
        if openList[bestIndex] == end:
            # reached end, need to create path then exit
            path = deque()
            node = openList[bestIndex]
            while node != None:
                path.appendleft(node.getPosition)
                node = node.getParent()
            return
        # add neighbors of best looking vertex if they aren't in the closed list
        for node in openList[i].getVisibleNeighbors:
            inClosed = False
            for nodeTwo in closedList:
                if node == nodeTwo:
                    inClosed = True
            if not inClosed:
                node.setParent(openList[bestIndex])
                node.setStartDistance(node.getParent().getStartDistance() + node.getPosition().distanceTo(node.getParent().getPosition()))
                node.setEndDistance(node.getPosition().distanceTo(end.getPosition()))
                openList.append(node)
        closedList.append(openList[bestIndex])
        openList.remove(openList[bestIndex])