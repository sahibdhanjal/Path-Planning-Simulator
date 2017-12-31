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
Provides a script to extract results from the output text file
"""


import sys


def main():

	try:
		inFile = open(sys.argv[1], 'r')
	except Exception, e:
		print 'A valid input file was not passed'

	volume = []
	visitednow = []
	redundancy = []
	infoGain = []
	totalMoves = []
	completionTime = 0
	# Number of times clustering was performed
	timesClustered = 0

	i = 1
	lines = inFile.readlines()
	for line in lines:
		currentLine = line.strip().split()
		# print currentLine
		if currentLine[0] == 'volume:':
			try:
				volume.append(int(currentLine[1]))
			except:
				print 'value of attribute volume missing from line', i, '(', line, ')'
			timesClustered += 1
		elif currentLine[0] == 'visitednow' or currentLine[0] == 'visitednow:':
			try:
				visitednow.append(int(currentLine[1]))
				# infoGain.append(int(currentLine[1]))	## Comment this
			except:
				print 'value of attribute visitednow missing from line', i, '(', line, ')'
		elif currentLine[0] == 'redundancy:' or currentLine[0] == 'redundancy':
			try:
				redundancy.append(int(currentLine[1]))
			except:
				print 'value of attribute redundancy missing from line', i, '(', line, ')'
			# Measuring the completion time based on the number of times the word
			# 'redundancy' occurs in the document is purely a design decision
			# It was so chosen because, Arjun's log files do not print 'visitednow'
			# They only print the information gain
			completionTime += 1
		elif currentLine[0] == 'totalMoves:':
			try:
				totalMoves.append(int(currentLine[1]))
			except:
				print 'value of attribute volume missing from line', i, '(', line, ')'
		elif currentLine[0] == 'visited':
			infoGain.append(int(currentLine[1]))	## Uncomment this
			pass
		else:
			pass


		i += 1

	# print 'volume:', volume
	# print 'visitednow:', visitednow
	# print 'redundancy:', redundancy

	if len(totalMoves) > 0:
		totalMoves = totalMoves[len(totalMoves) - 1]
		# print 'totalMoves:', totalMoves
	else:
		totalMoves = 0

	if totalMoves == 0:
		totalMoves = completionTime * 8

	# Compute information gain
	## Uncomment the following lines
	for i in range(len(visitednow)):
		if i == 0:
			infoGain.append(visitednow[0])
			continue
		infoGain.append(visitednow[i] - visitednow[i-1])

	# print 'infoGain:', infoGain
	totalRedundancy = sum(redundancy)
	# print 'totalRedundancy:', totalRedundancy
	
	if totalMoves != 0:
		percentageRedundancy = totalRedundancy * 100 / totalMoves
		# print 'percentageRedundancy:', percentageRedundancy
	else:
		# print 'percentageRedundancy: cannot be computed without knowing totalMoves'
		pass

	# print 'timesClustered:', timesClustered

	# print 'completionTime:', completionTime

	# Respond to command-line queries
	for i in range(2, len(sys.argv)):
		if sys.argv[i] == '-h':
			print 'time -> completionTime'
			print 'totalRedundancy -> total number of redundant moves'
			print 'redundancy -> number of redundant moves in each iteration (prints a list)'
			print 'totalRedundancy -> total number of redundant moves'
			print 'percentageRedundancy -> percentage of redundant moves'
			print 'infoGain -> information gain (prints a list)'
			print 'sumVisited -> cumulative sum of the number of cells visited (prints a list)'
			print 'volume -> volume of cells being considered for allocation (prints a list)'
			print 'totalMoves -> total number of moves made'

		elif sys.argv[i] == 'time':
			print 'completionTime:', completionTime
		elif sys.argv[i] == 'totalRedundancy':
			print 'totalRedundancy:', totalRedundancy
		elif sys.argv[i] == 'redundancy':
			for i in range(len(redundancy)):
				print redundancy[i]
		elif sys.argv[i] == 'totalRedundancy':
			print 'totalRedundancy:', totalRedundancy
		elif sys.argv[i] == 'percentageRedundancy':
			print 'percentageRedundancy:', percentageRedundancy
		elif sys.argv[i] == 'infoGain':
			for i in range(len(infoGain)):
				print infoGain[i]
		elif sys.argv[i] == 'sumVisited':
			for i in range(len(visitednow)):
				print visitednow[i]
		elif sys.argv[i] == 'volume':
			for i in range(len(volume)):
				print volume[i]
		elif sys.argv[i] == 'totalMoves':
			print 'totalMoves:', totalMoves
		elif sys.argv[i] == 'timesClustered':
			print 'timesClustered:', timesClustered


if __name__ == '__main__':

	main()