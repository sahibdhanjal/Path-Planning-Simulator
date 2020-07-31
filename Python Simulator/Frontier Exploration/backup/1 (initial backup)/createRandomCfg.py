import sys
import random

f = open('random.config', 'w')
f.write('height 50\n')
f.write('width 50\n')
numRobots = 'numRobots ' + str(sys.argv[1]) + '\n'
f.write(numRobots)
numRobots = int(sys.argv[1])
f.write('R 500\n')
f.write('baseX 0\n')
f.write('baseY 0\n')
f.write('initLocs\n')
initLocs = []
for i in range(numRobots):

	tempX = random.randint(0, 49)
	tempY = random.randint(0, 49)
	if (tempX, tempY) in initLocs:
		thereFlag = True
		while thereFlag == True:
			tempX = random.randint(0, 49)
			tempY = random.randint(0, 49)
			if (tempX, tempY) in initLocs:
				thereFlag = True
			else:
				thereFlag = False
	initLocs.append((tempX, tempY))
	initLoc = str(tempX) + ' ' + str(tempY) + '\n'
	f.write(initLoc)
f.close()