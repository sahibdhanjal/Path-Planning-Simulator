# The MIT License (MIT)

# Copyright (c) 2014 INSPIRE Lab, BITS Pilani

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
Provides methods that can be used to read in gridworlds from a configuration file of specified format.
"""


import sys


# The ConfigFileReader class
class ConfigFileReader:

	def __init__(self, fileName):

		self.fileName = fileName


	# Method to read the config file and return important parameters
	def readCfg(self):

		# First check if the filename is valid, i.e., if it ends with a .config
		l = len(self.fileName)
		if self.fileName[l-7:] != '.config':
			print 'Invalid configuration file. The filename of a valid configuration file is *.config'
			sys.exit(-1)

		cfgFile = open(self.fileName, "r")
		lines = cfgFile.readlines()
		cfgFile.close()

		initLocsFlag = False
		obstacleFlag = False
		height = 0
		width = 0
		numRobots = 0
		R = 0
		baseX = 0
		baseY = 0
		initLocs = []
		obstacles = []
		# Used in counting the number of initLocs read
		tempCount = 0

		for i in range(len(lines)):
			currentLine = lines[i].strip().split()
			# print currentLine

			# Ignore comments and empty lines
			if currentLine == []:
				pass
			elif currentLine[0] == '#':
				pass
			elif initLocsFlag == True:
				temp = []
				temp.append(int(currentLine[0]))
				temp.append(int(currentLine[1]))
				initLocs.append(temp)
				tempCount += 1
				# initLocs.append([int(currentLine[0]), int(currentLine[1])])
				if tempCount == numRobots:
					initLocsFlag = False
			elif obstacleFlag == True:
				temp = []
				temp.append(int(currentLine[0]))
				temp.append(int(currentLine[1]))
				obstacles.append(temp)
				# obstacles.append([int(currentLine[0]), int(currentLine[1])])
			elif currentLine[0] == 'height':
				height = int(currentLine[1])
			elif currentLine[0] == 'width':
				width = int(currentLine[1])
			elif currentLine[0] == 'numRobots':
				numRobots = int(currentLine[1])
			elif currentLine[0] == 'R':
				R = int(currentLine[1])
			elif currentLine[0] == 'baseX':
				baseX = int(currentLine[1])
			elif currentLine[0] == 'baseY':
				baseY = int(currentLine[1])
			elif currentLine[0] == 'initLocs':
				initLocsFlag = True
			elif currentLine[0] == 'obstacles':
				obstacleFlag = True
			else:
				print 'Invalid configuration file syntax'
				if initLocsFlag == True:
					print 'Check the number of initLocs provided in the configuration file.'
				return -1, height, width, numRobots, R, baseX, baseY, initLocs, obstacles

		return 0, height, width, numRobots, R, baseX, baseY, initLocs, obstacles