import AStar
import GridWorld

height=10;width=10;
obstacles=[(3,3),(2,3)]
gridworld = GridWorld.GridWorld(height, width, obstacles)

astar = AStar.AStar()

path, arr = astar.aStarSearch(gridworld, (1,1),(9,9))

print(path,arr)