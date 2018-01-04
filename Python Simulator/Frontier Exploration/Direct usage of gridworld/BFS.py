from Datatypes import *

class Dijkstra:
    def __init__(self):
        pass

    def aStarSearch(self,gridworld,start,goal):
        frontier=Queue()
        frontier.put(start)
        path={}
        cost={}
        path[start]=None
        cost[start]=0

        while not frontier.isEmpty():
            current = frontier.get()

            if current==goal:
                break

            for next in gridworld.get8Neighbors(current):
                if next not in cost:
                    frontier.put(next)
                    path[next]=current
                    cost[next]=cost[current]+1

        actualpath = []
        current = goal
        while current != start:
            actualpath.append(current)
            current = path[current]
        return actualpath,path,cost