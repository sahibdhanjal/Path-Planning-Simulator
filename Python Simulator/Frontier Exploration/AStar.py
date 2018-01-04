from Datatypes import *

class AStar:
	def __init__(self):
		pass
	
	def heuristic(self, a, b, n=4):
		dx=abs(b[0]-a[0])
		dy=abs(b[1]-a[1])

		# Manhattan Heuristic
		if n==1:
			return dx+dy

		# Euclidean Heuristic
		elif n==2:
			return (dx**2 + dy**2)**0.5

		# Chebychev Heuristic
		elif n==3:
			return max(dx,dy)

		# Octile Heuristic
		else: 
			return dx+dy+(2**0.5-2)*min (dx,dy)

	def aStarSearch(self,gridworld,start,goal):
		frontier=PriorityQueue()
		frontier.put(start,0)

		path={};cost={};v=1
		path[start]=None;cost[start]=0
		flag=0

		while not frontier.isEmpty():
			current = frontier.get()

			if current==goal:
				flag=1
				break

			for next in gridworld.get8Neighbors(current):
				if next[0]==current[0] or next[1] == current[1]:
					newcost=cost[current]+5
				else:
					newcost=cost[current]+7

				if next not in cost or newcost<cost[next]:
					cost[next]=newcost
					p = newcost + self.heuristic(goal,next)
					v+=1
					frontier.put(next,p)
					path[next]=current
		if flag==0:
			path[goal]=current
			cost[goal]=newcost

		actualpath = []
		current = goal
		while current != start:
			actualpath.append(current)
			current = path[current]
		print('AStar:',v,len(actualpath),cost[goal])
		return actualpath,path,cost