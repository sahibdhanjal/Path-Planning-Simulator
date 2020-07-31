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
Provides an implementation of the Communicative Exploration algorithm for a fixed base station.
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

# The Yamauchi class
class Yamauchi:

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
		# Flag to determine if printing of logs is required
		self.printLog = True

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
		unallocated = self.frontier
		isJobless = False
		allotted_frontier = ()

		# Assign to each robot its nearest frontier cell
		for i in range(self.numRobots):

			nearestDist = self.infinity
			curX = self.robots[i].curX
			curY = self.robots[i].curY

			for cell in unallocated:

				# print 'unallocated', unallocated
				if self.aStarFlag == True:
					path, dist = self.astar.aStarSearch(self.gridworld, (curX, curY), (cell[0], cell[1]))
					dist = dist[(cell[0], cell[1])]
				else:
					dist = abs(curX - cell[0]) + abs(curY - cell[1])

				if dist < nearestDist:
					nearestDist = dist
					allotted_frontier = cell
					allottedNow = True

			try:
				unallocated.remove(allotted_frontier)
			except Exception, e:
				pass

				# VERY FAST!!!
				# if allottedNow == True:
				# 	# print 'allotted_frontier', allotted_frontier
				# 	unallocated.remove(allotted_frontier)
				# 	allottedNow = False


			"""
			Move the ith robot closer to its allotted frontier
			"""
			# If all cells in the robot's assigned cluster have been explored, the robot waits in the same cell
			if isJobless == True:
				# genmax stores the command that is given to a robot
				# Kindly do not worry about the variable naming style
				# It was all Arjun's creativity and it shall be fixed soon
				shouldRecluster = True
				genmax = 8

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

		return cmd
			
		
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
		
		# Perform frontier allocation		
		bestCfgc = self.allocateFrontiers()

		self.executeBestCfgc(bestCfgc)
		# time.sleep(1)

		# Print the final statistics
		self.printFinalStats()
