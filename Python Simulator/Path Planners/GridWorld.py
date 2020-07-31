import Cell

class GridWorld:

	def __init__(self, width, height, obstacles):
		self.width = width
		self.height = height
		self.cells = [[Cell.Cell(i, j) for j in range(width)] for i in range(height)]
		self.obstacles = obstacles
		for obstacle in self.obstacles:
			self.cells[obstacle[0]][obstacle[1]].obstacle = True

	def isstart(self,item):
		self.cells[item[0]][item[1]].start = True

	def isgoal(self,item):
		self.cells[item[0]][item[1]].goal = True

	def mark(self,item):
		self.cells[item[0]][item[1]].visited = True

	def markjump(self,item):
		self.cells[item[0]][item[1]].isjp = True

	def markpath(self,item):
		self.cells[item[0]][item[1]].ispath = True

	def get4Neighbors(self, id):
		(curX, curY) = id
		if self.cells[curX][curY].obstacle == True:
			return []

		neighbors = [(curX - 1, curY), (curX, curY + 1), (curX + 1, curY), (curX, curY - 1)]
		neighbors = filter(self.inBounds, neighbors)
		neighbors = filter(self.passable, neighbors)
		return neighbors

	def get8Neighbors(self, id):
		(curX, curY) = id
		if self.cells[curX][curY].obstacle == True:
			return []

		neighbors = [(curX - 1, curY), (curX - 1, curY + 1), (curX, curY + 1), (curX + 1, curY + 1), (curX + 1, curY), (curX + 1, curY - 1), (curX, curY - 1), (curX - 1, curY - 1)]
		neighbors = filter(self.inBounds, neighbors)
		neighbors = filter(self.passable, neighbors)
		return neighbors

	def inBounds(self, id):
		(x, y) = id
		return 0 <= x < self.height and 0 <= y < self.width

	def passable(self, id):
		(x, y) = id
		return not (self.cells[x][y].obstacle)