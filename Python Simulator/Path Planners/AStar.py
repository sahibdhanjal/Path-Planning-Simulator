from Datatypes import *
import sys

frontier = PriorityQueue()
start = set()
goal = set()
explored = {}
cost = {}

class AStar:
	def __init__(self):
		pass

	def heuristic(self, a, b, n=1):
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

	def search(self,gridworld,begin,end):
		global frontier
		global start, goal, explored, cost

		start = begin ; goal = end
		frontier.put(start,0)

		explored[start]=None
		cost[start]=0

		gridworld.mark(start)
		gridworld.markpath(start)
		
		
		if self.proceed(gridworld) == 1:
			self.makepath()
			return False
		else:
			self.proceed(gridworld)
			return True
		
	def proceed(self,gridworld):
		global frontier
		global start, goal, explored, cost
		
		if frontier.isEmpty():
			return 1
		
		else:
			current=frontier.get()

			if current==goal:
				return 1
				# sys.exit()
			for next in gridworld.get8Neighbors(current):
				if next[0]==current[0] or next[1] == current[1]:
					newcost=cost[current]+5
				else:
					newcost=cost[current]+7

				if next not in cost:
					cost[next]=newcost
					priority = newcost + self.heuristic(next,goal)
					print(newcost,priority)
					frontier.put(next, priority)
					gridworld.mark(next)
					explored[next]=current

		return 0

	def makepath(self,gridworld):
		global goal, explored,cost,start
		path = []
		current = goal
		while current != start:
			path.append(current)
			gridworld.markpath(current)
			current = explored[current]
		path.reverse() ; path=[start]+path
		return path,explored,cost