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
Class to hold the grid world that represents the environment
"""


import Cell


# The GridWorld class
class GridWorld:

	def __init__(self, width, height, obstacles):

		# Store width and height
		self.width = width
		self.height = height
		# Create an array of grid cells
		self.cells = [[Cell.Cell(i, j) for j in range(width)] for i in range(height)]
		# Initialize the locations of the obstacles
		self.obstacles = obstacles
		for obstacle in self.obstacles:
			self.cells[obstacle[0]][obstacle[1]].obstacle = True


	# Method to check if (x, y) lies in the bounds of the grid
	def inBounds(self, id):

		(x, y) = id
		return 0 <= x < self.height and 0 <= y < self.width


	# Method to check if (x, y) is an obstacle cell or not
	def passable(self, id):

		(x, y) = id
		return not (self.cells[x][y].obstacle)# or self.cells[x][y].occupied) 


	# Method to get the 4-neighbors of the current cell identified by (curX, curY)
	def get4Neighbors(self, id):

		(curX, curY) = id

		# If the current cell is an obstacle, return an empty list
		if self.cells[curX][curY].obstacle == True:
			return []

		neighbors = [(curX - 1, curY), (curX, curY + 1), (curX + 1, curY), (curX, curY - 1)]
		# First, some filters
		neighbors = filter(self.inBounds, neighbors)
		neighbors = filter(self.passable, neighbors)
		# Then, return
		return neighbors


	# Method to get the 8-neighbors of the current cell identified by (curX, curY)
	def get8Neighbors(self, id):

		(curX, curY) = id

		# If the current cell is an obstacle, return an empty list
		if self.cells[curX][curY].obstacle == True:# or self.cells[curX][curY].occupied == True:
			return []

		neighbors = [(curX - 1, curY), (curX - 1, curY + 1), (curX, curY + 1), (curX + 1, curY + 1), (curX + 1, curY), (curX + 1, curY - 1), (curX, curY - 1), (curX - 1, curY - 1)]
		# First, some filters
		neighbors = filter(self.inBounds, neighbors)
		neighbors = filter(self.passable, neighbors)

		# Then, return
		return neighbors


	# Method to get the 8-neighbors of the current cell identified by (curX, curY)
	def get8Neighbors2(self, id):

		robotLocs = []
		for i in range(self.height):
			for j in range(self.width):
				if self.cells[i][j].occupied == True:
					robotLocs.append((i, j))

		(curX, curY) = id

		# If the current cell is an obstacle, return an empty list
		if self.cells[curX][curY].obstacle == True:# or self.cells[curX][curY].occupied == True:
			return []

		neighbors = [(curX - 1, curY), (curX - 1, curY + 1), (curX, curY + 1), (curX + 1, curY + 1), (curX + 1, curY), (curX + 1, curY - 1), (curX, curY - 1), (curX - 1, curY - 1)]
		# First, some filters
		neighbors = filter(self.inBounds, neighbors)
		neighbors = filter(self.passable, neighbors)

		neighbors = [item for item in neighbors if item not in robotLocs]
		# Then, return
		return neighbors


	def getcmd(self, nextX, nextY, curX, curY):
		# if self.cells[nextX][nextY].occupied == True:	##
			# cmd = 8 									##
		if curX == nextX and curY == nextY:
			cmd=8
		elif curX == nextX - 1 and curY == nextY:
			cmd = 6
		elif curX == nextX - 1 and curY == nextY - 1:
			cmd = 7
		elif curX == nextX and curY == nextY - 1:
			cmd=0
		elif curX == nextX + 1 and curY == nextY + 1:
			cmd = 3
		elif curX == nextX + 1 and curY == nextY:
			cmd = 2
		elif curX == nextX + 1 and curY == nextY - 1:
			cmd = 1
		elif curX == nextX and curY == nextY + 1:
			cmd=4
		elif curX == nextX - 1 and curY == nextY + 1:
			cmd = 5
		#print 'getting cmd'
		return cmd

	# Method to compute the next location given a command
	def getNextPos(self, curX, curY, command):

		# Variables to hold the next location
		nextX = 0
		nextY = 0

		if command == 0:
			if curY == self.width - 1:
				nextX = curX
				nextY = curY
			else:
				nextX = curX
				nextY = curY + 1

		elif command == 1:
			if curY == self.width - 1 or curX == 0:
				nextX = curX
				nextY = curY
			else:
				nextX = curX - 1
				nextY = curY + 1

		elif command == 2:
			if curX == 0:
				nextX = curX
				nextY = curY
			else:
				nextX = curX - 1
				nextY = curY

		elif command == 3:
			if curY == 0 or curX == 0:
				nextX = curX
				nextY = curY
			else:
				nextX = curX - 1
				nextY = curY - 1

		elif command == 4:
			if curY == 0:
				nextX = curX
				nextY = curY
			else:
				nextX = curX
				nextY = curY - 1

		elif command == 5:
			if curY == 0 or curX == self.height - 1:
				nextX = curX
				nextY = curY
			else:
				nextX = curX + 1
				nextY = curY - 1

		elif command == 6:
			if curX == self.height -1:
				nextX = curX
				nextY = curY
			else:
				nextX = curX + 1
				nextY = curY

		if command == 7:
			if curX == self.height - 1 or curY == self.width - 1:
				nextX = curX
				nextY = curY
			else:
				nextX = curX + 1
				nextY = curY + 1

		if command == 8:
			nextX = curX
			nextY = curY

		# if self.cells[nextX][nextY].occupied == True:	##
			# nextX = curX								##
			# nextY = curY								##


		return nextX, nextY


	# Method to check if a move is possible (i.e., if the new cell is already occupied or is an obstacle)
	def checkCommand(self, curX, curY, command):

		# If we're not moving to a new cell, first compute the new position given the command
		if command != 8:
			nextX, nextY = self.getNextPos(curX, curY, command)
			# If the next location is out of bounds, getNextPos returns curX, curY
			# In that case, the command is impossible
			if nextX == curX and nextY == curY:
				return False
			# In case the next position is possible, check if that position is an obstacle
			if self.cells[nextX][nextY].obstacle == True:
				return False
			# Otherwise, the command is possible

		# In case of command 8, it is always possible, assuming the initial location was possible
		return True
