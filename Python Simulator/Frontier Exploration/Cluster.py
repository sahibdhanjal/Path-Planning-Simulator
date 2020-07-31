class Cluster(object):

	def __init__(self, x, y):

		self.x = x
		self.y = y
		self.allotted = -1
		# Distance from the cluster's centroid to the presently allocated robot
		self.dist = 0
		# Arjun forgot to clean this up
		self.robot = 0
		# Denotes if the allocated (or reallocated) robot has already entered this cluster for exploration
		self.isInside = False
