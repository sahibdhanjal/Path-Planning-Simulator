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
import CommExplore
import ConfigFileReader
import GridWorld
import Hungarian
import kmeans
import Robot

from collections import defaultdict


def main():

	if len(sys.argv) == 2:
		configFileName = sys.argv[1]
	else:
		configFileName = "freeworld.config"
	
	cfgReader = ConfigFileReader.ConfigFileReader(configFileName)
	
	ret, height, width, numRobots, R, baseX, baseY, initLocs, obstacles = cfgReader.readCfg()
	if ret == -1:
		print 'readCfg() Unsuccessful!'
		sys.exit(-1)
	else:
		print 'Read the config file', configFileName

	k = 2
	T = 100000

	algo = CommExplore.CommExplore(height, width, obstacles, numRobots, initLocs, T)

	astar = AStar.AStar()

	path, cost = astar.aStarSearch(algo.gridworld, (0, 0), (6, 6))
	cost = cost[(6, 6)]
	print 'cost:', cost


if __name__ == '__main__':
	main()