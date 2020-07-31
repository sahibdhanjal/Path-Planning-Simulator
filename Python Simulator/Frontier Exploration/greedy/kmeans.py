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

def centroidnp(size, *points):
	x_coords = [t[0] for p in points for t in p]
	y_coords = [t[1] for p in points for t in p]
	
	#print x_coords
	#print y_coords
	#print size
	centroid_x = sum(x_coords)/size
	centroid_y = sum(y_coords)/size
	return [centroid_x, centroid_y]

class kmeans:
	def __init__(self):
		self.freeclusters=[]
		pass

	def Kmeanscluster(self, frontiers, no_robots):
		from cluster import KMeansClustering
		self.freeclusters=[]
		cl = KMeansClustering(frontiers)
		clusters = cl.getclusters(no_robots)
		#print clusters
		centroids=[]
		for i in range(no_robots):
			gen = centroidnp(len(clusters[i]),clusters[i])
			centroids.append(gen)
			genagain = Cluster.Cluster(centroids[i][0], centroids[i][1])
			genagain.occupied = False
			self.freeclusters.append(genagain)
		return self.freeclusters, clusters
		

