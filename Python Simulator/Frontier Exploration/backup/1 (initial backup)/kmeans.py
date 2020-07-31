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

import numpy
import cluster
import Cluster


"""
Computes the centroid, given the set of points in a cluster
"""
def centroidnp(points):

	x_coords = [t[0] for t in points]
	y_coords = [t[1] for t in points]

	centroid_x = sum(x_coords)/len(points)
	centroid_y = sum(y_coords)/len(points)
	return [centroid_x, centroid_y]


"""
Computes clusters, given a list of data points, using K-Means
"""
class kmeans:

	def __init__(self):
		self.freeclusters = []
		pass

	def Kmeanscluster(self, frontiers, no_robots):
		
		from cluster import KMeansClustering
		self.freeclusters = []
		# Perform clustering
		cl = KMeansClustering(frontiers)
		clusters = cl.getclusters(no_robots)
		
		# Compute centroids
		centroids=[]
		for i in range(no_robots):
			# Cheap hack (If the algorithm returns only a tuple convert it into a list, 
				# because centroidnp takes as input only a list)
			if type(clusters[i]) is tuple:
				clusters[i] = [clusters[i]]
			temp = centroidnp(clusters[i])
			centroids.append(temp)
			# Store each centroid in its corresponding Cluster object
			clusterObject = Cluster.Cluster(centroids[i][0], centroids[i][1])
			clusterObject.occupied = False
			self.freeclusters.append(clusterObject)
		
		return self.freeclusters, clusters
		