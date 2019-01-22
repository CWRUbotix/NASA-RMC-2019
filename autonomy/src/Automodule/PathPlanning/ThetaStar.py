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
            while node is not None:
                path.appendleft(node.getPosition)
                node = node.getParent()
            return
        # add neighbors of best looking vertex if they aren't in the closed list
        for node in openList[bestIndex].getVisibleNeighbors:
            # iterate through closed list to check if node has already been considered
            inClosed = False
            for nodeTwo in closedList:
                if node == nodeTwo:
                    inClosed = True
            tempScore = openList[bestIndex].getStartDistance() + node.getPosition().distanceTo()(openList[bestIndex].getPosition())
            tempScore += node.getPosition().distanceTo(end.getPosition())
            inOpen = False
            # iterate through open list to check if node can be reached in a better way
            for nodeTwo in openList:
                if node == nodeTwo:
                    inOpen = True
                    if tempScore < nodeTwo.getStartDistance() + nodeTwo.getEndDistance():
                        nodeTwo.setParent(openList[bestIndex])
                        node.setStartDistance(node.getParent().getStartDistance() + node.getPosition().distanceTo(
                            node.getParent().getPosition()))
                        node.setEndDistance(node.getPosition().distanceTo(end.getPosition()))
            # if node is new, add it to the open set
            if not inClosed and not inOpen:
                node.setParent(openList[bestIndex])
                node.setStartDistance(node.getParent().getStartDistance() + node.getPosition().distanceTo(node.getParent().getPosition()))
                node.setEndDistance(node.getPosition().distanceTo(end.getPosition()))
                openList.append(node)
        closedList.append(openList[bestIndex])
        openList.remove(openList[bestIndex])

function theta * (start, goal)
    gScore(start): = 0
    parent(start): = start
    open: = {}
    open.insert(start, gScore(start) + heuristic(start))
    closed: = {}
    while open is not empty
        s: = open.pop()
        if s = goal
           return reconstruct_path(s)
         closed.push(s)
        for each neighbor of s
            if neighbor not in closed
                if neighbor not in open
                    gScore(neighbor): = infinity
             parent(neighbor): = Null
            update_vertex(s, neighbor)
    return Null

function update_vertex(s, neighbor)
    if line_of_sight(parent(s), neighbor)
        if gScore(parent(s)) + c(parent(s), neighbor) < gScore(neighbor)
        gScore(neighbor): = gScore(parent(s)) + c(parent(s), neighbor)
        parent(neighbor): = parent(s)
            if neighbor in open
                open.remove(neighbor)
            open.insert(neighbor, gScore(neighbor) + heuristic(neighbor))
    else
        if gScore(s) + c(s, neighbor) < gScore(neighbor)
                gScore(neighbor): = gScore(s) + c(s, neighbor)
            parent(neighbor): = s
            if neighbor in open
                open.remove(neighbor)
            open.insert(neighbor, gScore(neighbor) + heuristic(neighbor))

function reconstruct_path(s)
    total_path = {s}
    if parent(s) != s
        total_path.push(reconstruct_path(parent(s)))
    else
        return total_path
