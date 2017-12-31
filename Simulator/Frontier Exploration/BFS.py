from Datatypes import *

class BFS:
    def __init__(self):
        pass

    def BFSsearch(self,gridworld,start,goal):
        frontier=Queue()
        frontier.put(start)
        path={}
        cost={}
        path[start]=None
        cost[start]=0
        v=1

        while not frontier.isEmpty():
            current = frontier.get()

            if current==goal:
                break

            for next in gridworld.get8Neighbors(current):
                if next not in cost:
                    frontier.put(next)
                    v+=1
                    path[next]=current
                    cost[next]=cost[current]+6

        actualpath = []
        current = goal
        while current != start:
            actualpath.append(current)
            current = path[current]
        print('BFS:',v,len(actualpath),cost[goal])
        return actualpath,path,cost