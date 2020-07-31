class Cell(object):
	def __init__(self, x, y):
		self.x = x
		self.y = y
		self.occupied = False
		self.obstacle = False
		self.visited = False
		self.ispath = False
		self.isprinted = False
		self.start = False
		self.goal = False