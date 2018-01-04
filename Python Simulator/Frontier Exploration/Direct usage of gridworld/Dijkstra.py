from Datatypes import *

class Dijkstra:
	def __init__(self):
		pass

	def aStarSearch(self,gridworld,start,goal):
		frontier=PriorityQueue()
		frontier.put(start,0)
		path={}
		cost={}
		path[start]=None
		cost[start]=0

		while not frontier.isEmpty():
			current = frontier.get()

			if current==goal:
				break

			for next in gridworld.get8Neighbors(current):
				if next[0]==current[0] or next[1] == current[1]:
					newcost=cost[current]+1
				else:
					newcost=cost[current]+1.4

				if next not in cost or newcost<cost[next]:
					cost[next]=newcost
					priority = newcost
					frontier.put(next, priority)
					path[next]=current

		actualpath = []
		current = goal
		while current != start:
			actualpath.append(current)
			current = path[current]
		return actualpath,path,cost