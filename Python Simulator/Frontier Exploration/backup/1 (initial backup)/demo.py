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
Provides a demo of the Frontier Clustering algorithm.
"""


from math import floor
import sys
from time import sleep
from Tkinter import Tk, Canvas, Frame, BOTH


import AStar
import Burgard
import CAC
import Faigl
import FrontierClusters
import ConfigFileReader
import GridUI
import Yamauchi


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

	# algo = Faigl.Faigl(height, width, obstacles, numRobots, initLocs, T)
	# algo = FrontierClusters.FrontierClusters(height, width, obstacles, numRobots, initLocs, T)
	algo = CAC.CAC(height, width, obstacles, numRobots, initLocs, T)
	# algo = Yamauchi.Yamauchi(height, width, obstacles, numRobots, initLocs, T)
	# algo = Burgard.Burgard(height, width, obstacles, numRobots, initLocs, T)

	# algo.printGrid()
	# print ''
	# print ''
	# cfgc = algo.generateCfgcPopulation()

	# for j in range(T):
	# 	algo.runOneIterV2()

	# timeTaken = algo.printVisitedStatus()
	# if timeTaken == 0:
	# 	return T
	# return timeTaken

	
	# Code that takes care of the GUI
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

	# Runs one iteration of the algorithm and updates the GUI
	def run():

		algo.runOneIter()
		gui.redraw(height, width, cellSize, algo.gridworld, algo.robots, algo.frontier)
		root.after(1, run)

	root.after(1, run)
	root.mainloop()


if __name__ == '__main__':
	
	main()
