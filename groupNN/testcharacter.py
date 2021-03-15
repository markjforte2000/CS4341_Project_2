from math import sqrt
# This is necessary to find the main code
import sys

sys.path.insert(0, '../bomberman')
# Import necessary stuff
from entity import CharacterEntity
from colorama import Fore, Back


class TestCharacter(CharacterEntity):

    def do(self, wrld):
        # Your code here
        # Check state first
        entities = self.getEntities(wrld)
        walls, bombs, explosions, monsters, exitTile = entities
        print("walls: " + str(walls))
        print("bombs: " + str(bombs))
        print("explosions: " + str(explosions))
        print("exitTile" + str(exitTile))
        # TODO: Identifying states, when to use astar, identifying blocking lines, bombing, evading
        # TODO: Expectimax? Q?
        path = self.aStar(wrld, entities)
        moveSquare = path[1]
        moveDx = moveSquare[0] - self.x
        moveDy = moveSquare[1] - self.y
        self.move(moveDx, moveDy)

    # Goes through the entire board and returns a list of tuples
    # representing the entities present in the world, of the
    # given type, where entities[i][0] = x and entities[i][1] = y.
    def getEntities(self, wrld):
        print("getting entities")
        bombs = []
        walls = []
        explosions = []
        monsters = []
        exits = []
        for i in range(wrld.width()):
            for j in range(wrld.height()):
                if wrld.wall_at(i, j):
                    walls.append((i, j))
                if wrld.bomb_at(i, j):
                    bombs.append((i, j))
                if wrld.explosion_at(i, j):
                    explosions.append((i, j))
                if wrld.monsters_at(i, j):
                    monsters.append((i, j))
                if wrld.exit_at(i, j):
                    exits.append((i, j))
        return walls, bombs, explosions, monsters, next(iter(exits or []), None)

    def aStar(self, wrld, entities, target=None,  start=None):
        exitx = entities[4][0]
        exity = entities[4][1]
        # if no defined target, set it to the exit
        if target is None:
            target = (exitx, exity)
        # if no defined start, set it to the current position.
        if start is None:
            start = (self.x, self.y)
        path = []
        if start == target:
            # start is target, so the path to target is nothing.
            return path
        # Format for frontier entries:
        # (x, y), f
        # Having f here is important for sorting the nodes.
        frontier = [list(((start[0], start[1]), 0))]
        # cameFrom will be a 2d array of tuples (x, y)
        cameFrom = []
        # costSoFar will be the distance traveled so far. This is g.
        costSoFar = []
        # closed will be the list of nodes that have already been visited
        closed = []
        for i in range(wrld.height()):
            # accessing these is the "wrong" way around
            # closed[y][x] instead of xy
            cameFrom.insert(0, [None]*wrld.width())
            costSoFar.insert(0, [float("inf")]*wrld.width())
            closed.insert(0, [False]*wrld.width())
        costSoFar[start[1]][start[0]] = 0
        print("starting aStar search")
        while len(frontier) > 0:
            # Sort by f-value for which to pop first
            frontier.sort(key=lambda entry: entry[1])
            print("frontier: " + str(frontier))
            # take the first value from the sorted list, this is what needs to be looked at
            current = frontier.pop(0)
            closed[current[0][1]][current[0][0]] = True
            # check if we have reached the target yet
            if(current[0][0], current[0][1]) == target:
                break
            # Going through the child nodes of the current node.
            # A* algorithm will:
            #  - if the node is a wall, monster, bomb, explosion, or is in closed, ignore.
            #  - if the node is not in the frontier, add, compute costSoFar (g), h, and f, and add cameFrom
            #  - if the node is in the frontier, check if the path to this square from current is better
            #    than the previous path. if so, change parent, g, f.
            # print("Calling validChildren on current: " + str(current))
            validChildren = self.getValidNodes(wrld, current[0][0], current[0][1], entities, closed)
            for child in validChildren:
                # childdx = abs(current[0][1] - child[0])
                # childdy = abs(current[0][0] - child[1])
                childCost = 1 + costSoFar[current[0][1]][current[0][0]]
                heuristic = self.pythagoras(child[0] - target[0], child[1] - target[1])
                print("child x: " + str(child[0]) + " y: " + str(child[1]) + " heuristic: " + str(heuristic))
                # check if it's in the frontier
                if self.isInFrontier(frontier, child):
                    wasLess = False
                    # check if the path from curent to child is better than what we have
                    if childCost < costSoFar[child[1]][child[0]]:
                        # set costSoFar and cameFrom for the child.
                        costSoFar[child[1]][child[0]] = childCost
                        cameFrom[child[1]][child[0]] = current[0]
                        index = self.getFrontierIndex(frontier, child)
                        tempListObject = list(frontier[index])
                        tempListObject[1] = childCost + heuristic
                        frontier[index] = tempListObject
                        wasLess = True
                    print("Child was in frontier, was less: " + str(wasLess))
                else:
                    # not in the frontier, so add it.
                    costSoFar[child[1]][child[0]] = childCost
                    cameFrom[child[1]][child[0]] = current[0]
                    frontier.append(list((child, childCost + heuristic)))
                    print("child was not in frontier, added")

        # Rebuild the path now from exit node.
        # if the exit has no cameFrom entry, then the algorithm failed to make it there.
        print("rebuilding path")
        if cameFrom[target[1]][target[0]] is None:
            # Failed to find a path to the exit.
            return path
        else:
            # Backtrack from the exit to find the path.
            currentNode = list(target)
            path.insert(0, currentNode)
            while not currentNode == list(start):
                if not (cameFrom[currentNode[1]][currentNode[0]] is None):
                    path.insert(0, list(cameFrom[currentNode[1]][currentNode[0]]))
                    currentNode = list(cameFrom[currentNode[1]][currentNode[0]])
                else:
                    # somehow the cameFrom of a node that isn't the start had none as its came from? what?
                    return []
            # if we exited the loop, then we must have a path from start to target. Nice!
            return path

    def getFrontierIndex(self, frontier, child):
        justTuples = self.getJustTuples(frontier)
        return justTuples.index(child)

    def isInFrontier(self, frontier, child):
        justTuples = self.getJustTuples(frontier)
        return child in justTuples

    def getJustTuples(self, frontier):
        justTuples = []
        for tup in frontier:
            justTuples.append(tup[0])
        return justTuples

    def getValidNodes(self, wrld, x, y, entities, closed):
        # print("getValidNodes on x: " + str(x) + ", y: " + str(y))
        validNodes = []
        for i in range(-1, 2):
            for j in range(-1, 2):
                # Check if the node is out of bounds
                if i == 0 and j == 0:
                    # That's the parent node. no need.
                    continue
                if(0 <= (x + i) < wrld.width()) and (0 <= (y + j) < wrld.height()):
                    # it's in bounds, now see if its valid (aka not a wall, monster, or bomb/explosion)
                    if (not ((x+i, y+j) in entities[0])) and (not ((x+i, y+j) in entities[1])):
                        if (not ((x+i, y+j) in entities[2])) and (not (x+i, y+j) in entities[3]):
                            # it's none of those, last thing to check is if its closed or not.
                            try:
                                if not closed[y + j][x + i]:
                                    validNodes.append(list((x + i, y + j)))
                            except IndexError:
                                print("tried to access closed[" + str(y) + " + " + str(j) + "][" + str(x) + " + " + str(i)
                                      + "] and got an exception.")
                                print("world values are width: " + str(wrld.width()) + " height: " + str(wrld.height()))
                                exit(-1)
        return validNodes

    def heuristic(self, x, y, exitTile):
        # simple heuristic for now, the distance between the exit tile and the current.
        return self.pythagoras((x - exitTile[0]), (y - exitTile[1]))

    # for calculating simple straight line distances because having to write it out
    # multiple times is annoying.
    def pythagoras(self, dx, dy):
        return sqrt((dx ** 2) + (dy ** 2))
