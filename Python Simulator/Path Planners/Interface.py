from math import floor
from time import sleep
from Tkinter import Tk, Canvas, Frame, BOTH

'''
Color Palette:

http://www.colourlovers.com/palette/92095/Giant_Goldfish

border=#737373
beach storm=#ffffff --- Unexplored
Clean Pond Water=#afeeee --- Visited
yellow_crumbs=#ffff00 --- Path
brown_raisins=#808080 --- Obstacles

'''
class Interface(Frame):
	def __init__(self, parent, height, width, cellSize, grid):

		Frame.__init__(self, parent)
		self.parent = parent
		self.initialize(height, width, cellSize, grid)


	def initialize(self, height, width, cellSize, grid):

		self.parent.title('Simulator')
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
			# Obstacle
			if grid.cells[xIdx][yIdx].obstacle == True:
				self.canvas.create_rectangle(curX, curY, curX + cellSize, curY + cellSize, outline = '#737373', fill = '#808080', width = 2)	

			# Start
			elif grid.cells[xIdx][yIdx].start == True:
				self.canvas.create_rectangle(curX, curY, curX + cellSize, curY + cellSize, outline = '#737373', fill = 'red', width = 2)

			# Goal
			elif grid.cells[xIdx][yIdx].goal == True:
				self.canvas.create_rectangle(curX, curY, curX + cellSize, curY + cellSize, outline = '#737373', fill = 'green', width = 2)
			
			# Unexplored Cell
			else:
				self.canvas.create_rectangle(curX, curY, curX + cellSize, curY + cellSize, outline = '#737373', fill = '#ffffff', width = 2)
			
			curX = curX + cellSize
			if curX == endX and curY != endY:
				curX = startX
				xIdx += 1
				curY = curY + cellSize
				yIdx = 0
				continue
			elif curX == endX and curY == endY:
				break
			rectIdx += 1
			yIdx += 1

		self.canvas.pack(fill = BOTH, expand = 1)


	# Method to redraw the positions of the robots and the frontier
	def redraw(self, height, width, cellSize, grid):

		self.parent.title('Simulator')
		self.pack(fill = BOTH, expand = 1)

		self.canvas.delete('all')

		startX = cellSize
		startY = cellSize
		endX = startX + (cellSize * width)
		endY = startY + (cellSize * height)

		curX = startX
		curY = startY
		xIdx = 0
		yIdx = 0

		while curX != endX and curY != endY:
			# Obstacle
			if grid.cells[xIdx][yIdx].obstacle == True:
				self.canvas.create_rectangle(curX, curY, curX + cellSize, curY + cellSize, outline = '#737373', fill = '#808080', width = 2)	
			
			# Start
			elif grid.cells[xIdx][yIdx].start == True:
				self.canvas.create_rectangle(curX, curY, curX + cellSize, curY + cellSize, outline = '#737373', fill = 'red', width = 2)

			# Visited Cell
			elif grid.cells[xIdx][yIdx].visited == True and grid.cells[xIdx][yIdx].ispath == False :
				self.canvas.create_rectangle(curX, curY, curX + cellSize, curY + cellSize, outline = '#737373', fill = '#afeeee', width = 2)

			# Path Cell
			elif grid.cells[xIdx][yIdx].ispath == True and grid.cells[xIdx][yIdx].visited == True:
				self.canvas.create_rectangle(curX, curY, curX + cellSize, curY + cellSize, outline = '#737373', fill = '#ffff00', width = 2)
				grid.cells[xIdx][yIdx].isprinted = True			


			# Goal
			elif grid.cells[xIdx][yIdx].goal == True:
				self.canvas.create_rectangle(curX, curY, curX + cellSize, curY + cellSize, outline = '#737373', fill = 'green', width = 2)

			# Unexplored Cell
			else:
				self.canvas.create_rectangle(curX, curY, curX + cellSize, curY + cellSize, outline = '#737373', fill = '#ffffff', width = 2)
				
			
			curX = curX + cellSize

			if curX == endX and curY != endY:
				curX = startX
				xIdx += 1
				curY = curY + cellSize
				yIdx = 0
				continue

			elif curX == endX and curY == endY:
				break

			yIdx += 1

		self.canvas.pack(fill = BOTH, expand = 1)