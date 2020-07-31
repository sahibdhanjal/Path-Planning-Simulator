infile = open('freeworld_pruned.txt')
lines = infile.readlines()
avgTime = 0.0
avgMetric = 0.0
count = 0
for line in lines:
	currentLine = line.strip().split()
	time = int(currentLine[0])
	metric = float(currentLine[1])
	avgTime += time
	avgMetric += metric
	count += 1
avgTime = avgTime / float(count)
avgMetric = avgMetric / float(count)
print 'avgTime:', avgTime
print 'avgMetric:', avgMetric