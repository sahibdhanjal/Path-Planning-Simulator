# The MIT License (MIT)

# Copyright (c) 2015 INSPIRE Lab, BITS Pilani

# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.


"""
Provides an implementation of the A* algorithm to find the shortest path between two nodes.
Since we use an optimal heuristic (here, Manhattan distance), the path returned is guaranteed to be optimal.
"""


import PriorityQueue


# The AStar class
class AStar:

	def __init__(self):
		pass


	# Method that computes the value of the heuristic between two locations a, b in the gridworld
	# Here the heuristic is taken to be the Manhattan distance from a to b, ignoring obstacles
	def heuristic(self, a, b):

		(x1, y1) = a
		(x2, y2) = b
		return abs(x1 - x2) + abs(y1 + y2)


	# Method that runs the A* algorithm and returns the computed path as well as its cost
	def aStarSearch(self, gridworld, start, goal):
		print(start,goal)
		# Frontier of the A* algorithm. Not to be confused with the frontier that we use in exploration
		# Initialize the frontier and put the start cell on it
		frontier = PriorityQueue.PriorityQueue()
		frontier.put(start, 0)

		# Declare and initialize other variables to store the computed path and cost
		path = {}
		cost = {}
		path[start] = None
		cost[start] = 0
		flag=0
		# Compute the next location to be added to the path
		while not frontier.isEmpty():

			current = frontier.get()

			if current == goal:
				flag=1
				break

			for next in gridworld.get8Neighbors(current):
				
				newCost = cost[current] + 1

				if next not in cost or newCost < cost[next]:
					cost[next] = newCost
					priority = newCost + self.heuristic(goal, next)
					frontier.put(next, priority)
					path[next] = current
		if flag == 0:
			path[goal]=current
			cost[goal]=newCost
		return path, cost

	
