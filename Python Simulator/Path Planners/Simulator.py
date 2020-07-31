from ConfigFileReader import *
import time,sys,Init
from Dijkstra import *
from Interface import *
from AStar import *
from BFS import *

start = (0,10)
goal = (2,48)

t = 2	# Set time to display GUI

if len(sys.argv)>1:
	configFileName = sys.argv[1]
	algo = sys.argv[2] ; s = sys.argv[3] ; g = sys.argv[4]

	if algo == 'bfs':
		algoflag=1
	elif algo == 'dijkstra':
		algoflag=2
	elif algo == 'astar':
		algoflag=3
	else:
		raise("No such algorithm present!")

	x,y=s.split(',')
	x=int(x)
	y=int(y)
	start=(x,y)

	xg,yg=g.split(',')
	xg=int(xg) ; yg=int(yg)
	goal=(xg,yg)

else:
	configFileName = "freeworld_large.config"
	algoflag = 3 ; start = (0,0) ; goal = (49,49)



ret, height, width, numRobots, R, baseX, baseY, initLocs, obstacles=ConfigFileReader(configFileName).readCfg()

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

initializer = Init.Initialize(height, width, obstacles)
gui = Interface(root, height, width, cellSize, initializer.gridworld)
guiHeight = str((height + 2) * cellSize)
guiWidth = str((width + 2) * cellSize)
xOffset = str(xoffset)
yOffset = str(yoffset)
geometryParam = guiWidth + 'x' + guiHeight + '+' + xOffset + '+' + yOffset
root.geometry(geometryParam)

if initializer.gridworld.cells[goal[0]][goal[1]].obstacle==True:
	raise('Goal is an obstacle!')

if initializer.gridworld.cells[start[0]][start[1]].obstacle==True:
	raise('Start is an obstacle!')

		
initializer.gridworld.isstart(start)
initializer.gridworld.isgoal(goal)
		
flag=False ; p=[]; cost = 0

def run():
	global flag,p,cost
	
	gui.redraw(height, width, cellSize, initializer.gridworld)

	if algoflag == 1:
		path = BFS().search(initializer.gridworld, start, goal)
	elif algoflag == 2:
		path = Dijkstra().search(initializer.gridworld, start, goal)
	else:
		path = AStar().search(initializer.gridworld, start, goal)
	
	if flag== False:
		if path== True and initializer.gridworld.cells[goal[0]][goal[1]].visited==False:
			if algoflag == 1:
				BFS().search(initializer.gridworld, start, goal)
			elif algoflag == 2:
				Dijkstra().search(initializer.gridworld, start, goal)
			else:
				AStar().search(initializer.gridworld, start, goal)

		else:
			if algoflag == 1:
				p, explored, cost= BFS().makepath(initializer.gridworld)
			elif algoflag == 2:
				p, explored, cost= Dijkstra().makepath(initializer.gridworld)
			else:
				p, explored, cost= AStar().makepath(initializer.gridworld)
			flag=True
		gui.redraw(height, width, cellSize, initializer.gridworld)
	
	else:
		print('Path found!')
		print('The path is:',p)
		print('The cost is:',cost[goal])
		print('GUI will exit in:',t,'seconds')
		time.sleep(t)
		sys.exit()

	root.after(1,run)
root.after(1,run)
root.mainloop()