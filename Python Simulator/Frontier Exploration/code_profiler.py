'''
Profiler for Path Planning Algorithms
@author - Sahib Singh Dhanjal
'''

import GridWorld
import ConfigFileReader

import BFS


start=(0,0)
goal=(49,49)

configFileName = "office.config"

ret, height, width, numRobots, R, baseX, baseY, initLocs, obstacles = ConfigFileReader.ConfigFileReader(configFileName).readCfg()

gridworld = GridWorld.GridWorld(height, width, obstacles)

path4,actual,cost4=BFS.BFS().BFSsearch(gridworld,start,goal)
print('Profiling Done')