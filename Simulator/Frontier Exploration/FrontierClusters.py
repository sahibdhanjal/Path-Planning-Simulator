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
Provides an implementation of the Frontier Clustering algorithm.
"""


import math
import random
import sys
import time

import AStar
import Cluster
import GridWorld
import Hungarian
import kmeans
import Robot

from collections import defaultdict
# The FrontierClusters class
class FrontierClusters:

	"""
	height and width specify the dimensions of the environment
	obstacles is a list of locations which are to be initialized as obstacles
	R specifies the range of communication
	numRobots specifies the number of robot objects to be initialized
	initLocs specifies the initial locations for each of the robots
	k specifies the size of the population of configuration changes
	T specifies the number of time steps to run the simulation for
	base specifies the coordinates of the base station
	"""
	def __init__(self, height, width, obstacles, numRobots, initLocs, T = 10):

		# Initialize the grid world
		self.gridworld = GridWorld.GridWorld(height, width, obstacles)
		self.centroid = []
		self.cluster = kmeans.kmeans()
		# Initialize a list of robots
		self.robots = [Robot.Robot(j+1, -1, -1) for j in range(numRobots)]
		# Initialize the starting location of each robot
		i = 0
		#self.allotted=Cell.Cell(0,0)
		for initLoc in initLocs:
			# If any robot is placed out of bounds or on an obstacle, print an error message and exit
			currentPoint = (initLoc[0], initLoc[1])
			if not self.gridworld.inBounds(currentPoint) or not self.gridworld.passable(currentPoint):
				print 'Initial location', currentPoint, 'is not possible'
				sys.exit(-1)
			# Otherwise, modify the current location of the robot to currentPoint
			self.robots[i].setLocation(initLoc[0], initLoc[1])
			# Update that particular grid cell's occupied status
			self.gridworld.cells[initLoc[0]][initLoc[1]].occupied = True
			self.gridworld.cells[initLoc[0]][initLoc[1]].visited = True
			i += 1


		# Initialize other parameters of the algorithm
		# Height of the Grid
		self.height = height
		# Width of the Grid
		self.width = width
		# List of Clusters (obtained using K-Means clustering)
		self.frontierClusters=[]
		# Number of Robots
		self.numRobots = numRobots
		# Parameter for adaptive K-Means
		self.K = self.numRobots
		# Time steps for which the algorithm is run
		self.T = T
		# Variable to indicate whether reclustering should be performed
		self.reclusterFlag = True
		# Centroids of clusters
		self.centroids = []
		# Number of time steps elapsed
		self.t = 0
		# Time taken to exhaust the frontier
		self.completionTime = 0
		# Set to True on completion
		self.completedFlag = False
		# List of Frontier Cells
		self.frontier = []
		# New Positions of each of the Robots
		self.newPos = []
		# Population of Configuration Changes
		self.cfgc = []
		# Number of Stalls (Used only for simulating Rooker's work)
		self.stalls = 0
		# Keeps track of whether the Final Stats were displayed
		self.printedFinalStats = False
		# Keeps track of Possible Configurations
		self.possible = []
		# Keeps track of Number of Cells Visited
		# (Initialize this to self.numRobots, since the starting locations of the robots are considered visited)
		self.visited = self.numRobots
		self.sumNewVisited = numRobots
		# Flag to switch between A* and Manhattan distance
		self.aStarFlag = False
		# Define value for infinity
		self.infinity = 10000000
		# Flag to switch between Hungarian and Greedy assignment
		self.hungarianFlag = True
		# Flag to switch between Greedy and Random Motion Planner
		self.randomMotionPlan = False
		# Variable to store the number of robots that do not make a move in the current iteration
		self.currentEights = 0
		# Variable to store the total number of moves made by the robots
		self.totalMoves = 0
		# Variable to store the volume of cells allocated
		self.volume = 0
		# Variable to store the percentage of allotted cells explored
		self.exploredCells = 0
		# Variable to store the number of times clustering occurred
		self.timesClustered = 0
		# Defines the threshold percentage
		self.thresholdPercentage = 1.0
		# Flag to determine if reallocation is required
		self.reallocateFlag = False
		# Flag to determine if printing of logs is required
		self.printLog = False

		# We also initialize an instance of AStar, which helps us in computing Manhattan distance
		self.astar = AStar.AStar()


	# Method to print the current gridworld to the output descriptor
	def printGrid(self):

		## Comment this later
		frontier = self.computeFrontier()
		##
		print 'occupied cells:'
		for i in range(self.height):
			for j in range(self.width):
				if self.gridworld.cells[i][j].occupied == True:
					print i, j
		print 'robot locations:'
		for robot in self.robots:
			print robot.curX, robot.curY

		for i in range(self.height):
			for j in range(self.width):
				# If the current cell is an obstacle, print #
				if self.gridworld.cells[i][j].obstacle == True:
					sys.stdout.write(' # ')
				# If the current cell is occupied by a robot, print its id
				elif self.gridworld.cells[i][j].occupied == True:
					robotId = 0
					for robot in self.robots:
						if robot.curX == i and robot.curY == j:
							robotId = robot.id
					temp = ' ' + str(robotId) + ' '
					sys.stdout.write(temp)
				# If the current cell is a frontier, print a |
				elif (i, j) in frontier:
					sys.stdout.write(' | ')
				# Otherwise, print -
				else:
					if self.gridworld.cells[i][j].visited == True:
						sys.stdout.write(' . ')
					else:
						sys.stdout.write(' - ')
			sys.stdout.write('\n')


	# Method to print the status of each cell to the output descriptor
	def printVisitedStatus(self):

		visited = 0
		visitable = self.height * self.width

		for i in range(self.height):
			for j in range(self.width):
				# If the current cell is an obstacle, print #
				if self.gridworld.cells[i][j].visited == True:
					sys.stdout.write(' 1 ')
					visited += 1
				# If the current cell is a frontier, print a |
				else:
					sys.stdout.write(' 0 ')
					if self.gridworld.cells[i][j].obstacle == True:
						visitable -= 1
			sys.stdout.write('\n')

		print 'visited:', visited, ' of ', visitable
		print 'stalls:', self.stalls

		return self.completionTime


	# Method to print the final statistics to the output descriptor
	# The final stats should be printed only once
	# i.e., either when T steps have elapsed or when the frontier is empty
	def printFinalStats(self, force = 0):

		# if self.t != self.T:
		# 	if self.completedFlag == False or self.printedFinalStats == True:
		# 		return

		# print 'Time taken:', self.t
		visitednow=0
		visitable = self.height * self.width

		for i in range(self.height):
			for j in range(self.width):
				# If the current cell is an obstacle, print #
				if self.gridworld.cells[i][j].visited == True:
					visitednow += 1
				# If the current cell is a frontier, print a |
				else:
					if self.gridworld.cells[i][j].obstacle == True:
						visitable -= 1

		metric = self.visited / visitable

		# print (visitednow - self.visited)
		if self.printLog == True:
			print 'visitednow', visitednow	##
		redundancy = (self.numRobots - self.currentEights) - (visitednow - self.visited)
		if self.printLog == True:
			print 'redundancy:', redundancy	##
		# print self.currentEights

		self.exploredCells += (visitednow - self.visited)

		self.visited = visitednow
		numMoves = self.numRobots - self.currentEights
		self.totalMoves += numMoves
		if self.printLog == True:
			print 'totalMoves:', self.totalMoves	##
			print 'self.exploredCells:', self.exploredCells	##

		self.printedFinalStats = True
		return


	# Method to compute the frontier
	def computeFrontier(self):

		frontier = []

		# Iterate over all cells in the grid
		for i in range(self.height):
			for j in range(self.width):
				# We compute 8-neighbors for only those cells that haven't been visited or are obstacles
				# Only such cells are possible candidates for the frontier
				if self.gridworld.cells[i][j].visited == False and self.gridworld.cells[i][j].obstacle == False:
					point = (i, j)
					neighbors = self.gridworld.get8Neighbors(point)
					# Now we see if there is at least one neighbor of the current cell which has been visited
					# In such a case, the current cell would become a frontier cell
					frontierFlag = False
					for nbhr in neighbors:
						if self.gridworld.cells[nbhr[0]][nbhr[1]].visited == True:
							frontierFlag = True

					if frontierFlag == True:
						frontier.append((i, j))

		return frontier


	# Method to delete lone cells
	def deleteLoneCells(self):

		# Kill cells using the rules from Conway's Game of Life
		for i in range(self.height):
			for j in range(self.width):
				
				if self.gridworld.cells[i][j].visited == True:
					continue
				else:
					visitedNeighbors = 0
					neighbors = self.gridworld.get8Neighbors((i, j))
					for neighbor in neighbors:
						if self.gridworld.cells[neighbor[0]][neighbor[1]].visited == True:
							visitedNeighbors += 1
					if visitedNeighbors == len(neighbors):
						self.gridworld.cells[i][j].visited = True

		frontier = self.computeFrontier()

		return frontier


	# Method to compute the new locations of each robot, given a command vector
	def getNewPositions(self, cmd):

		newPos = []

		for i in range(self.numRobots):

			nextX, nextY = self.gridworld.getNextPos(self.robots[i].curX, self.robots[i].curY, cmd[i])
			newPos.append((nextX, nextY))

		return newPos


	# Method to check if a given configuration is possible
	# We return an integer that describes the nature of the impossibility of a configuration
	# 0 denotes that all robots move into non-obstacle cells
	# 1 denotes that two robots occupy the same cell
	# 2 denotes that one robot encounters an obstacle
	def isCfgPossible(self, cfg):

		# We first compute the new positions and see if two next positions coincide
		# newPos = self.getNewPositions(cfg)
		if any(self.newPos.count(element) > 1 for element in self.newPos) == True:
			return 1

		# Now we check if some robot encounters an obstacle
		retval = 0
		for i in range(self.numRobots):
			if self.gridworld.checkCommand(self.robots[i].curX, self.robots[i].curY, cfg[i]) == False:
				retval = 2

		# Otherwise, the configuration is possible
		return retval


	def allocateFrontiers(self):
		
		cmd = []
		# Allocate robots to clusters
		if self.reallocateFlag == True:

			"""
			If hungarianFlag is set to True, perform Hungarian assignment
			Else, perform greedy assignment.
			"""

			# Hungarian assignment
			if self.hungarianFlag == True:
				# Determine the cost matrix
				costMatrix = []

				# Get the distance of each robot to its nearest cell in the cluster
				for i in range(self.numRobots):
					
					closestDist = self.infinity
					cost = []
					for cluster in self.frontierClusters:
						
						closestDistToCluster = self.infinity
						for point in cluster:
							
							if self.aStarFlag == True:
								path, distanceToPoint = self.astar.aStarSearch(self.gridworld, (self.robots[i].curX, self.robots[i].curY), (point[0], point[1]))
								distanceToPoint = distanceToPoint[(point[0], point[1])]
							else:
								distanceToPoint = abs(self.robots[i].curX - point[0]) + abs(self.robots[i].curY - point[1])

							if distanceToPoint < closestDistToCluster:
								closestDistToCluster = distanceToPoint
						
						cost.append(closestDistToCluster)

					costMatrix.append(cost)
				
				costMatrix = zip(*costMatrix)
				# print 'costMatrixHere:', costMatrix, self.K
				# for centroid in self.centroids:

				# 	# Denotes the distance from each robot to the current centroid
				# 	cost = []

				# 	for robot in self.robots:

				# 		# Note the confusing notation
				# 		# manhattan has been used for bot manhattan and A* distances
				# 		if self.aStarFlag == True:
				# 			path, manhattan = self.astar.aStarSearch(self.gridworld, (robot.curX, robot.curY), (centroid.x, centroid.y))
				# 			manhattan = manhattan[(centroid.x, centroid.y)]
				# 		else:
				# 			manhattan = abs(robot.curX - centroid.x) + abs(robot.curY - centroid.y)
				# 		cost.append(manhattan)

				# 	costMatrix.append(cost)

				if self.K < self.numRobots:

					additionalRows = self.numRobots - self.K
					cost = [0 for j in range(self.numRobots)]
					for i in range(additionalRows):
						costMatrix.append(cost)

				# if self.K < self.numRobots:
				# 	print 'costMatrix: ', costMatrix, 'self.K:', self.K

				"""Create an instance of the Hungarian assignment class"""
				hungarian = Hungarian.Hungarian()
				hungarian.calculate(costMatrix)
				results = hungarian.get_results()
				# print 'Total Potential: ', hungarian.get_total_potential()
				# if self.K < self.numRobots:
				# 	print 'Hungarian results:', results
				
				# Choose only those allotments where the id of the cluster is less than self.K
				if self.K < self.numRobots:
					def filterFunction(parameter):
						return parameter[0] < self.K-1
					results = filter(filterFunction, results)
					# print 'results:', results
				
				# Perform the assignment
				for result in results:
					self.centroids[result[0]].allotted = result[1]

				robotList = [i for i in range(self.numRobots)]
				
				for result in results:
					if result[1] in robotList:
						robotList.remove(result[1])
					
				# print 'robotList', robotList, self.K
				
				for robot in robotList:
					clusterObject = Cluster.Cluster(self.robots[robot].curX, self.robots[robot].curY)
					clusterObject.allotted = robot
					self.centroids.append(clusterObject)
					# print 'prevClusters:', self.frontierClusters
					self.frontierClusters.append([(self.robots[robot].curX, self.robots[robot].curY)])
					# if robot < self.K:
					# 	self.frontierClusters[robot] = ([(self.robots[robot].curX, self.robots[robot].curY)])
					# else:
					# 	self.frontierClusters.append([(self.robots[robot].curX, self.robots[robot].curY)])

				# print 'clusters:', self.frontierClusters

			# Greedy assignment
			else:
				
				totalPotential = 0
				# Iterate over the list of cluster centroids
				for j in range(self.numRobots):
				
					robotPos = (self.robots[0].curX, self.robots[0].curY)
					centroidPos = (self.centroids[j].x, self.centroids[j].y)
				
					if self.aStarFlag == True:
						path, tmp = self.astar.aStarSearch(self.gridworld, robotPos, (self.infinity, self.infinity))
						# tmp = tmp[(self.infinity, self.infinity)]
						tmp = self.infinity
					else:
						tmp = self.infinity
				
					# Iterate over the list of robots
					for i in range(self.numRobots):	
					
						robotPos = (self.robots[i].curX, self.robots[i].curY)
					
						if self.aStarFlag == True:
							path, tempdist = self.astar.aStarSearch(self.gridworld, robotPos, centroidPos)
							tempdist = tempdist[centroidPos]
						else:
							tempdist = abs(robotPos[0]-centroidPos[0]) + abs(robotPos[1]-centroidPos[1])
					
						allottedflag = False
						# Check if the robot has already been allotted a centroid
						for k in range(self.numRobots):
							if self.centroids[k].allotted==i:
								allottedflag = True
						# If it hasn't been allotted
						if tmp >= tempdist and allottedflag == False:
							tmp = tempdist
							self.centroids[j].allotted = i
							totalPotential += tmp

				# print 'Total Potential: ', totalPotential
		
		"""
		Take over from the victimized
		"""
		whoIsJobless = []
		# Find jobless robots
		for i in range(self.numRobots):

			allottedCluster = -1
			# Find the cluster to which the ith robot was allotted
			for j in range(self.numRobots):
				if self.centroids[j].allotted == i:
					allottedCluster = j
					break

			curX = self.robots[i].curX
			curY = self.robots[i].curY
			jobless = True
			isInside = False
			for point in self.frontierClusters[allottedCluster]:
				# Check if the cell has been visited already
				# If yes mark this as done
				if point[0] == curX and point[1] == curY:
					isInside = True
				if self.gridworld.cells[point[0]][point[1]].visited == False:
					jobless = False

			if jobless == True:
				whoIsJobless.append(i)

			self.centroids[allottedCluster].isInside = isInside

		# print 'whoIsJobless:', whoIsJobless
		if len(whoIsJobless) > 0:

			newRobot = -1
			for j in range(self.numRobots):

				if self.centroids[j].isInside == False:
				
					if self.aStarFlag == True:
						path, distance = self.astar.aStarSearch(self.gridworld, (0, 0), (self.infinity, self.infinity))
						# distance = distance[(self.infinity, self.infinity)]
						distance = self.infinity
					else:
						distance = self.infinity

					# Select closest jobless robot to cluster
					for joblessRobot in whoIsJobless:
						
						if self.aStarFlag == True:
							path, tempDistance = self.astar.aStarSearch(self.gridworld, (self.robots[joblessRobot].curX, self.robots[joblessRobot].curY), (self.centroids[j].x, self.centroids[j].y))
							tempDistance = tempDistance[(self.centroids[j].x, self.centroids[j].y)]
						else:
							tempDistance = abs(self.robots[joblessRobot].curX - self.centroids[j].x) + abs(self.robots[joblessRobot].curY - self.centroids[j].y)

						if tempDistance < distance:
							distance = tempDistance
							newRobot = joblessRobot

					# Compare it with the distance of the already allocated robot to the centroid
					currentlyAllotted = self.centroids[j].allotted
					if self.aStarFlag == True:
						path, currentDistance = self.astar.aStarSearch(self.gridworld, (self.robots[currentlyAllotted].curX, self.robots[currentlyAllotted].curY), (self.centroids[j].x, self.centroids[j].y))
						currentDistance = currentDistance[(self.centroids[j].x, self.centroids[j].y)]
					else:
						currentDistance = abs(self.robots[currentlyAllotted].curX - self.centroids[j].x) + abs(self.robots[currentlyAllotted].curY - self.centroids[j].y)
					# If it is closer than the currently allocated robot, it takes over
					if distance < currentDistance and newRobot != -1:
						oldAllotted = self.centroids[j].allotted
						for k in range(self.numRobots): 
							if self.centroids[k].allotted == newRobot:
								self.centroids[k].allotted = oldAllotted
						self.centroids[j].allotted = newRobot
						# print "Reallotted", newRobot, "to cluster", j


		"Motion planning"
		shouldRecluster = False
		for i in range(self.numRobots):

			# If randomMotionPlan is True, use random motion planning
			# i.e., visit each cluster cell in the order in which K-Means returned it
			if self.randomMotionPlan == True:

				isJobless = True
				"""
				Determine the cluster that was allotted the ith robot
				Store it in temp
				"""
				for j in range(self.numRobots):
					if self.centroids[j].allotted==i:
						temp=j
						break
				# print 'robot', i, 'given cluster', temp
				
				# Iterate over each frontier cell
				for cellgen in self.frontierClusters[temp]:
			
					# Mark the cell as not done
					thisisdone=0
				
					# Check if the cell has been visited already
					# If yes mark this as done
					for j in range(self.gridworld.height):
						for k in range(self.gridworld.width):
							if self.gridworld.cells[j][k].x == cellgen[0] and self.gridworld.cells[j][k].y == cellgen[1] and self.gridworld.cells[j][k].visited==True:
								thisisdone=1
					# If it has not been marked as done, allot this cell to the robot
					# The variable 'isJobless' denotes if the robots frontier cluster is jobless now
					if thisisdone==0:
						allotted_frontier=cellgen
						isJobless = False
						break

			# Otherwise, use the greedy motion plan
			else:
				
				isJobless = True
				# To each robot, allocate the closest cell in its allotted frontier
				# For that, initialize the least distance to infinity
				dist = self.infinity	## If any error occurs, maybe commenting this line would resolve it
				# Store the cluster number allotted to the ith robot
				for j in range(self.numRobots):
					if self.centroids[j].allotted == i:
						allottedCluster = j
						break
				# print 'robot', i, 'given cluster', j
				
				dists = []
				if self.aStarFlag == True:
					path, dist = self.astar.aStarSearch(self.gridworld, (0, 0), (self.infinity, self.infinity))
					dist = self.infinity
					# print 'dist:', dist
				else:
					dist = self.infinity
				# Iterate over each frontier cell in the ith robot's cluster
				for cellgen in self.frontierClusters[allottedCluster]:

					# Mark the cell as not done
					thisIsDone = 0

					robotPos = (self.robots[i].curX, self.robots[i].curY)
					cellPos = (cellgen[0], cellgen[1])

					# Check if the cell has been visited already
					# If yes mark this as done
					if self.gridworld.cells[cellgen[0]][cellgen[1]].visited == True:
						thisIsDone = 1
					
					# If it has not been marked as done, allot this cell to the robot
					# The variable 'isJobless' denotes if the robots frontier cluster is jobless now
					if thisIsDone == 0:
						if self.aStarFlag == True:
							path, tempDist = self.astar.aStarSearch(self.gridworld, robotPos, cellPos)
							tempDist = tempDist[cellPos]
						else:
							tempDist = abs(robotPos[0] - cellPos[0]) + abs(robotPos[1] - cellPos[1])
						dists.append(tempDist)
						if tempDist < dist:
							dist = tempDist
							allotted_frontier = cellgen
							isJobless = False
				
				# print 'dist:', dists, dist
			# print i, isJobless

			"""
			Move the ith robot closer to its allotted frontier
			"""
			# If all cells in the robot's assigned cluster have been explored, the robot waits in the same cell
			if self.volume > 0:
				exploredPercentage = self.exploredCells / (self.volume * 1.0)
				if self.printLog == True:
					print 'self.exploredCells:', self.exploredCells	##
				# print 'exploredPercentage:', exploredPercentage

			if self.thresholdPercentage <= 0.1:
				self.thresholdPercentage = 0.1
			else:
				# print '#########twenty-five iterations:', self.t / 25
				if self.t < 200:
					self.thresholdPercentage = 0.5
				else:
					if self.t > 600 and self.t < 800:
						self.thresholdPercentage = 0.5
					else:
						self.thresholdPercentage = 0.5 - 0.01*(self.t / 25)

			"""Uncomment these lines to revert to the old plan"""
			# # if isJobless == True or exploredPercentage >= self.thresholdPercentage:
			# if exploredPercentage >= self.thresholdPercentage:
			# 	# genmax stores the command that is given to a robot
			# 	# Kindly do not worry about the variable naming style
			# 	# It was all Arjun's creativity and it shall be fixed soon
			# 	shouldRecluster = True
			# 	genmax = 8
			# 	# self.exploredCells = 0	# ??

			if isJobless == True: # or exploredPercentage >= self.thresholdPercentage:
				genmax = 8
				shouldRecluster = True
				# if exploredPercentage >= self.thresholdPercentage:
					# shouldRecluster = True

			# Otherwise, it visits the cell
			else:

				# possCells stores the current 8-neighbors of the ith robot
				possCells=[]
				possCells = self.gridworld.get8Neighbors((self.robots[i].curX, self.robots[i].curY))
				# If using A*
				if self.aStarFlag == True:
					path, tmp = self.astar.aStarSearch(self.gridworld, possCells[0], allotted_frontier)
					tmp = tmp[allotted_frontier]
				# If A* is not being used, Manhattan distance is used
				else:
					tmp = abs(possCells[0][0]-allotted_frontier[0]) + abs(possCells[0][1]-allotted_frontier[1])
				# Here's yet another name from our creative genius (Arjun)
				# This variable is initialized with the first of its 8-neighbors
				thechosenone=possCells[0]
			
				# For each neighbor of the ith robot
				for nextcell in possCells:
					# If using A*
					if self.aStarFlag == True:
						path, tmp1 = self.astar.aStarSearch(self.gridworld, nextcell, allotted_frontier)
						tmp1 = tmp1[allotted_frontier]
					# If A* is not being used, Manhattan distance is used
					else:
						tmp1=abs(nextcell[0]-allotted_frontier[0]) + abs(nextcell[1]-allotted_frontier[1])
					
					# if tmp>=tmp1:	## Error ?
					if tmp1 < tmp:
						# path, tmp = self.astar.aStarSearch(self.gridworld, nextcell, allotted_frontier)
						tmp=tmp1;
						thechosenone = nextcell
				genmax=self.gridworld.getcmd(thechosenone[0], thechosenone[1], self.robots[i].curX, self.robots[i].curY)
			cmd.append(genmax)
		
		# print 'cmd:', cmd
		# for i in range(len(cmd)):
		# 	temp = str(cmd[i])
		# 	sys.stdout.write(temp)
		# 	sys.stdout.write(' ')
		# sys.stdout.write('\n')

		return cmd, shouldRecluster
			
		
	def executeBestCfgc(self, bestCfgc):

		i = 0

		for cmd in bestCfgc:
			tempX = self.robots[i].curX
			tempY = self.robots[i].curY
			if self.gridworld.checkCommand(tempX, tempY, cmd) == True:
				nextX, nextY = self.gridworld.getNextPos(tempX, tempY, cmd)
				self.gridworld.cells[tempX][tempY].occupied = False
				self.robots[i].curX = nextX
				self.robots[i].curY = nextY
				self.gridworld.cells[nextX][nextY].occupied = True
				self.gridworld.cells[nextX][nextY].visited = True
			i += 1



	# Run the algorithm for 1 iteration
	def runOneIter(self):

		# If t time steps have already expired, return
		self.t += 1
		if self.t >= self.T:
			if self.printedFinalStats == False:
				self.printFinalStats()
			return

		# Else, run the algorithm for one time step
		self.frontier = self.computeFrontier()
		if self.frontier == []:
			if self.completedFlag == False:
				self.completedFlag = True
				self.completionTime = self.t
				# print 'Completed in time', self.completionTime
				self.printFinalStats()
			return

		# self.frontier = self.deleteLoneCells()
		
		if len(self.frontier) < self.numRobots:
			self.K = len(self.frontier)
		else:
			self.K = self.numRobots

		# If reclusterFlag is true, then perform K-Means clustering
		if self.reclusterFlag == True:
			
			if self.printLog == True:
				print 'volume:', len(self.frontier)	##
			self.exploredCells = 0
			self.volume = len(self.frontier)

			if self.K != 1:
				self.centroids, self.frontierClusters = self.cluster.Kmeanscluster(self.frontier, self.K)
			# K-Means doesn't accept K = 1, so handle that case separately
			# by creating a centroid for that cell alone
			else:
				clusterObject = Cluster.Cluster(self.frontier[0][0], self.frontier[0][1])
				self.centroids = [clusterObject]
				self.frontierClusters = [self.frontier]
			self.timesClustered += 1

			# Calculating a few values in order to plot dispersion 
			# (not of any use to the algorithm)
			centroidOfCentroids = ()
			sumX = 0
			sumY = 0
			for centroid in self.centroids:
				sumX += centroid.x
				sumY += centroid.y
			if self.centroids != 0:
				tempX = int(sumX / len(self.centroids))
				tempY = int(sumY / len(self.centroids))
				# print tempX, tempY
				centroidOfCentroids = (tempX, tempY)
			sumOfDistances = 0
			for centroid in self.centroids:
				# sumOfDistances += (abs(centroid.x - centroidOfCentroids[0]) + abs(centroid.y - centroidOfCentroids[1]))
				sumOfDistances += (((centroid.x - centroidOfCentroids[0])*(centroid.x - centroidOfCentroids[0])) + ((centroid.y - centroidOfCentroids[1]) * (centroid.y - centroidOfCentroids[1])))
			print sumOfDistances

			self.reallocateFlag = True

		# Perform frontier allocation		
		bestCfgc, shouldRecluster = self.allocateFrontiers()
		self.currentEights = 0
		for cfg in bestCfgc:
			if cfg == 8:
				self.currentEights += 1

		# print shouldRecluster
		if shouldRecluster == True and self.K == self.numRobots:
			self.reclusterFlag = True
			return
		else:
			self.reclusterFlag = False
		# if shouldRecluster == True and self.K < self.numRobots:
		# 	bestCfgc, shouldRecluster = self.allocateFrontiers()
		# print self.t, shouldRecluster
		

		# self.reclusterFlag = True
		for i in range(self.numRobots):
			for cellgen in self.frontierClusters[i]:
				for j in range(self.gridworld.height):
					for k in range(self.gridworld.width):
						if self.gridworld.cells[j][k].x == cellgen[0] and self.gridworld.cells[j][k].y == cellgen[1] and self.gridworld.cells[j][k].visited==False:
							# self.reclusterFlag = False
							self.gridworld.cells[j][k].cluster=i
							break
		# 					#print 'not jobless'

		self.executeBestCfgc(bestCfgc)
		# time.sleep(1)

		# Print the final statistics
		self.printFinalStats()
