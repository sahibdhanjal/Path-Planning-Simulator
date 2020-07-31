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
Defines a Tkinter based GUI class for visualizing the simulation
"""


from math import floor
from time import sleep
from Tkinter import Tk, Canvas, Frame, BOTH


# The GridUI class
class GridUI(Frame):


	def __init__(self, parent, height, width, cellSize, grid, robots, frontier):

		Frame.__init__(self, parent)
		self.parent = parent
		self.initialize(height, width, cellSize, grid, robots, frontier)


	# Method to draw a grid of specified height and width
	def initialize(self, height, width, cellSize, grid, robots, frontier):

		self.parent.title('Grid')
		self.pack(fill = BOTH, expand = 1)

		self.canvas = Canvas(self)

		startX = cellSize
		startY = cellSize
		endX = startX + (cellSize * width)
		endY = startY + (cellSize * height)

		curX = startX
		curY = startY
		rectIdx = 0
		xIdx = 0
		yIdx = 0

		while curX != endX and curY != endY:
			
			# print 'x, y:', xIdx, yIdx
			# First, check if the current location corresponds to that of any robot
			robotFlag = False
			for robot in robots:
				if robot.curX == xIdx and robot.curY == yIdx:
					robotFlag = True
					self.canvas.create_rectangle(curX, curY, curX + cellSize, curY + cellSize, outline = '#0000FF', fill = '#00FF00', width = 2)
			# Then check if it corresponds to an obstacle
			if grid.cells[xIdx][yIdx].obstacle == True:
				self.canvas.create_rectangle(curX, curY, curX + cellSize, curY + cellSize, outline = '#0000FF', fill = '#000000', width = 2)	
			elif robotFlag == False:
				# Then check if it corresponds to a frontier cell
				frontierFlag = False
				for pt in frontier:
					if pt[0] == xIdx and pt[1] == yIdx:
						self.canvas.create_rectangle(curX, curY, curX + cellSize, curY + cellSize, outline = '#0000FF', fill = '#00FFFF', width = 2)
						frontierFlag = True

				if frontierFlag == False:
					if grid.cells[xIdx][yIdx].visited == True:
						self.canvas.create_rectangle(curX, curY, curX + cellSize, curY + cellSize, outline = '#0000FF', fill = '#FFFFFF', width = 2)
					else:
						self.canvas.create_rectangle(curX, curY, curX + cellSize, curY + cellSize, outline = '#0000FF', fill = '#777777', width = 2)
			
			curX = curX + cellSize
			if curX == endX and curY != endY:
				curX = startX
				xIdx += 1
				curY = curY + cellSize
				yIdx = 0
				# Move to the next iteration of the loop
				continue
			elif curX == endX and curY == endY:
				break
			rectIdx += 1
			yIdx += 1

		self.canvas.pack(fill = BOTH, expand = 1)


	# Method to redraw the positions of the robots and the frontier
	def redraw(self, height, width, cellSize, grid, robots, frontier):

		self.parent.title('Grid2')
		self.pack(fill = BOTH, expand = 1)

		# canvas = Canvas(self.parent)

		self.canvas.delete('all')

		startX = cellSize
		startY = cellSize
		endX = startX + (cellSize * width)
		endY = startY + (cellSize * height)

		curX = startX
		curY = startY
		rectIdx = 0
		xIdx = 0
		yIdx = 0

		while curX != endX and curY != endY:
			
			# print 'x, y:', xIdx, yIdx
			# First, check if the current location corresponds to that of any robot
			robotFlag = False
			if grid.cells[xIdx][yIdx].centroid == True:
				self.canvas.create_rectangle(curX, curY, curX + cellSize, curY + cellSize, outline = '#0000FF', fill = 'indian red', width = 2)
			else:
				
				for robot in robots:
					if robot.curX == xIdx and robot.curY == yIdx:
						robotFlag = True
						self.canvas.create_rectangle(curX, curY, curX + cellSize, curY + cellSize, outline = '#0000FF', fill = '#00FF00', width = 2)
				# Then check if it corresponds to an obstacle
				if grid.cells[xIdx][yIdx].obstacle == True:
					self.canvas.create_rectangle(curX, curY, curX + cellSize, curY + cellSize, outline = '#0000FF', fill = '#000000', width = 2)	
				elif robotFlag == False:
					# Then check if it corresponds to a frontier cell
					frontierFlag = False
					for pt in frontier:
						if pt[0] == xIdx and pt[1] == yIdx:
							if grid.cells[xIdx][yIdx].cluster==-1:
								self.canvas.create_rectangle(curX, curY, curX + cellSize, curY + cellSize, outline = '#0000FF', fill = '#00FFFF', width = 2)
							if grid.cells[xIdx][yIdx].cluster == 0:
								self.canvas.create_rectangle(curX, curY, curX + cellSize, curY + cellSize, outline = '#0000FF', fill = 'midnight blue', width = 2)
							if grid.cells[xIdx][yIdx].cluster == 1:
								self.canvas.create_rectangle(curX, curY, curX + cellSize, curY + cellSize, outline = '#0000FF', fill = 'peach puff', width = 2)
							if grid.cells[xIdx][yIdx].cluster == 2:
								self.canvas.create_rectangle(curX, curY, curX + cellSize, curY + cellSize, outline = '#0000FF', fill = 'gold', width = 2)
							if grid.cells[xIdx][yIdx].cluster == 3:
								self.canvas.create_rectangle(curX, curY, curX + cellSize, curY + cellSize, outline = '#0000FF', fill = 'slate blue', width = 2)
							if grid.cells[xIdx][yIdx].cluster == 4:
								self.canvas.create_rectangle(curX, curY, curX + cellSize, curY + cellSize, outline = '#0000FF', fill = 'royal blue', width = 2)
							if grid.cells[xIdx][yIdx].cluster == 5:
								self.canvas.create_rectangle(curX, curY, curX + cellSize, curY + cellSize, outline = '#0000FF', fill = 'red', width = 2)
							if grid.cells[xIdx][yIdx].cluster == 6:
								self.canvas.create_rectangle(curX, curY, curX + cellSize, curY + cellSize, outline = '#0000FF', fill = 'dark green', width = 2)
							if grid.cells[xIdx][yIdx].cluster == 7:
								self.canvas.create_rectangle(curX, curY, curX + cellSize, curY + cellSize, outline = '#0000FF', fill = 'yellow', width = 2)
							if grid.cells[xIdx][yIdx].cluster == 8:
								self.canvas.create_rectangle(curX, curY, curX + cellSize, curY + cellSize, outline = '#0000FF', fill = 'orange', width = 2)
							if grid.cells[xIdx][yIdx].cluster == 9:
								self.canvas.create_rectangle(curX, curY, curX + cellSize, curY + cellSize, outline = '#0000FF', fill = 'hot pink', width = 2)
							frontierFlag = True

					if frontierFlag == False:
						if grid.cells[xIdx][yIdx].visited == True:
							self.canvas.create_rectangle(curX, curY, curX + cellSize, curY + cellSize, outline = '#0000FF', fill = '#FFFFFF', width = 2)
						#else:
							#if grid.cells[xIdx][yIdx].cluster != -1:
							#	if grid.cells[xIdx][yIdx].cluster == 0:
							#		self.canvas.create_rectangle(curX, curY, curX + cellSize, curY + cellSize, outline = '#0000FF', fill = 'midnight blue', width = 2)
							#	if grid.cells[xIdx][yIdx].cluster == 1:
							#		self.canvas.create_rectangle(curX, curY, curX + cellSize, curY + cellSize, outline = '#0000FF', fill = 'peach puff', width = 2)
							#	if grid.cells[xIdx][yIdx].cluster == 2:
							#		self.canvas.create_rectangle(curX, curY, curX + cellSize, curY + cellSize, outline = '#0000FF', fill = 'dim gray', width = 2)
							#	if grid.cells[xIdx][yIdx].cluster == 3:
							#		self.canvas.create_rectangle(curX, curY, curX + cellSize, curY + cellSize, outline = '#0000FF', fill = 'slate blue', width = 2)
							#	if grid.cells[xIdx][yIdx].cluster == 4:
							#		self.canvas.create_rectangle(curX, curY, curX + cellSize, curY + cellSize, outline = '#0000FF', fill = 'royal blue', width = 2)
							#	if grid.cells[xIdx][yIdx].cluster == 5:
							#		self.canvas.create_rectangle(curX, curY, curX + cellSize, curY + cellSize, outline = '#0000FF', fill = 'red', width = 2)
							#	if grid.cells[xIdx][yIdx].cluster == 6:
							#		self.canvas.create_rectangle(curX, curY, curX + cellSize, curY + cellSize, outline = '#0000FF', fill = 'dark green', width = 2)
							#	if grid.cells[xIdx][yIdx].cluster == 7:
							#		self.canvas.create_rectangle(curX, curY, curX + cellSize, curY + cellSize, outline = '#0000FF', fill = 'gold', width = 2)
							#	if grid.cells[xIdx][yIdx].cluster == 8:
							#		self.canvas.create_rectangle(curX, curY, curX + cellSize, curY + cellSize, outline = '#0000FF', fill = 'orange', width = 2)
							#	if grid.cells[xIdx][yIdx].cluster == 9:
							#		self.canvas.create_rectangle(curX, curY, curX + cellSize, curY + cellSize, outline = '#0000FF', fill = 'hot pink', width = 2)
						else :
							self.canvas.create_rectangle(curX, curY, curX + cellSize, curY + cellSize, outline = '#0000FF', fill = '#777777', width = 2)
			
			curX = curX + cellSize
			if curX == endX and curY != endY:
				curX = startX
				xIdx += 1
				curY = curY + cellSize
				yIdx = 0
				# Move to the next iteration of the loop
				continue
			elif curX == endX and curY == endY:
				break
			rectIdx += 1
			yIdx += 1

		self.canvas.pack(fill = BOTH, expand = 1)
