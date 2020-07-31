import sys

class ConfigFileReader:

	def __init__(self, fileName):
		self.fileName = fileName

	def readCfg(self):
		l = len(self.fileName)
		
		# CHECK VALID CONFIG FILE NAME
		if self.fileName[l-7:] != '.config':
			print('Invalid configuration file. The filename of a valid configuration file is *.config')
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
			if currentLine == []:
				pass
			
			elif currentLine[0] == '#':
				pass
			
			elif initLocsFlag == True:
				initLocs.append((int(currentLine[0]) , int(currentLine[1])))
				tempCount += 1
				if tempCount == numRobots:
					initLocsFlag = False
			
			elif obstacleFlag == True:
				obstacles.append((int(currentLine[0]) , int(currentLine[1])))
			
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
				print('Invalid configuration file syntax')
				if initLocsFlag == True:
					print('Check the number of initLocs provided in the configuration file.')
				return -1, height, width, numRobots, R, baseX, baseY, initLocs, obstacles

		return 0, height, width, numRobots, R, baseX, baseY, initLocs, obstacles