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


from math import *
import random
import sys
import time
import GridUI
import AStar
import Cluster
import CAC
import Dijkstra
import AStar_arjun
import ConfigFileReader
import GridWorld
import Hungarian
import kmeans
import Robot
import JPS

from Tkinter import Tk, Canvas, Frame, BOTH

from collections import defaultdict


TIMER=True


def main():

	if len(sys.argv) == 2:
		configFileName = sys.argv[1]
	else:
		configFileName = "office.config"
	
	cfgReader = ConfigFileReader.ConfigFileReader(configFileName)
	
	ret, height, width, numRobots, R, baseX, baseY, initLocs, obstacles = cfgReader.readCfg()
	if ret == -1:
		print 'readCfg() Unsuccessful!'
		sys.exit(-1)
	else:
		print 'Read the config file', configFileName

	k = 2
	T = 100000

	algo = CAC.CAC(height, width, obstacles, numRobots, initLocs, T)
	

	if height <= 10:
		xoffset = 300
	else:
		xoffset = 100
	if width <= 10:
		yoffset = 300
	else:
		yoffset = 100

	maxScreenHeight = 700
	cellSize = int(floor(maxScreenHeight / (height + 2)))

	root = Tk()

	gui = GridUI.GridUI(root, height, width, cellSize, algo.gridworld, algo.robots, algo.frontier)
	guiHeight = str((height + 2) * cellSize)
	guiWidth = str((width + 2) * cellSize)
	xOffset = str(xoffset)
	yOffset = str(yoffset)
	geometryParam = guiWidth + 'x' + guiHeight + '+' + xOffset + '+' + yOffset
	root.geometry(geometryParam)


	ALGOFLAG=1



	if ALGOFLAG==1:
		astar = Dijkstra.Dijkstra()
		path,covered,cost = astar.aStarSearch(algo.gridworld, (0,0), (35, 40))
		cost = cost[(35,40)]
	if ALGOFLAG==2:
		astar = AStar.AStar()
		path,covered,cost = astar.aStarSearch(algo.gridworld, (0,0), (35, 40))
		cost = cost[(35,40)]
	if ALGOFLAG==3:
		astar = AStar_arjun.AStar()
		path, cost = astar.aStarSearch(algo.gridworld, (0,0), (35, 40))
		cost = cost[(35,40)]
	if ALGOFLAG==4:
		astar = JPS.JPS()
		covered=astar.jps(algo.gridworld, (0,0), (35,40))
		path,cost=astar.full_path(covered)
		print 'cost:', cost

	if TIMER==True:
		print("--- %s seconds ---" % (time.time() - start_time))
	
	def run():
		gui.redraw(height, width, cellSize, algo.gridworld, algo.robots, path)
		#gui.redraw(height, width, cellSize, algo.gridworld, algo.robots, covered)
		root.after(1,run)

	root.after(1,run)
	root.mainloop()
	
if __name__ == '__main__':
	if TIMER==True:
		start_time = time.time()
	main()
