import sys


def main():

	try:
		inFile = open(sys.argv[1], 'r')
	except Exception, e:
		print 'A valid input file was not passed'

	# We assume that the first line contains the number of robots
	# and the subsequent lines contain the redundancies incurred in each iteration
	numRobots = 0
	hist = []
	firstTime = True

	lines = inFile.readlines()
	# print lines
	for line in lines:
		currentLine = line.strip().split()
		if firstTime == True:
			numRobots = int(currentLine[0])
			firstTime = False
			for i in range(numRobots + 1):
				hist.append(0)
			continue
		hist[int(currentLine[0])] += 1	

	print 'hist:', hist


if __name__ == '__main__':
	main()