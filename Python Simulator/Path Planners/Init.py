import GridWorld

class Initialize:
	def __init__(self, height, width, obstacles):
		self.gridworld = GridWorld.GridWorld(height, width, obstacles)
		self.height = height
		self.width = width