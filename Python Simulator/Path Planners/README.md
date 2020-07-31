<h1>Simulator Usage:</h1>
<ol>
<li>Navigate to the folder and run the file named ConfigFileMaker.py</li>
<li>Click on the opened GUI to make obstacles. Once done doing this, click on “Set” and then “Exit” button.</li>
<li>By default the size of the map is given to be a grid of 20 x 20 cells. However this can be changed by changing the values of ROWS and COLUMNS in the above mentioned file. The name of the config file generated is “one.config” by default. You can change it by changing line 8 of the above mentioned file.</li>
<li>Once done run the Simulator.py file via terminal. The syntax is to be:
python Simulator.py <configfile name> <algo name> <start node> <goal node>
Eg. python Simulator.py one.config 1,0 10,10</li>

Program Structure:
1.	Datatypes.py - > Contains implementations of all the ADT’s such as queue, stack and priority queue.

2.	Cell.py - > A class that is used to initialize the basic cell on the grid and its properties. The properties are:
a.	(x,y) – the coordinates of the cell
b.	Occupied – flag to see if the cell is occupied or not
c.	Obstacle – flag to determine if the cell is an obstacle or not
d.	Visited - flag to determine if the cell has been previously visited or not
e.	ispath – flag to determine if the cell lies in the path of the robot
f.	isprinted - flag to determine if the cell has been marked or not when visited
g.	start – is the cell the starting node
h.	goal – is the cell a goal node

3.	Init.py -> takes as input the height, width and obstacle coordinates and returns an object representing the grid map

4.	ConfigFileMaker.py -> Makes a .config file in the required format using user input from a gui

5.	ConfigFileReader.py -> Reads the .config file and determines the characteristics of the map such as height, width, number of robots, start positions, and the obstacle positions

6.	Gridworld.py -> Gives functionality to the map by basic functions such as :
a.	isstart() – if the cell is a start node
b.	isgoal() – if the cell is a goal node
c.	mark() -  if the cell has been visited
d.	markjump() – if the cell has been marked as a jump point or not
e.	get4neighbors() – returns the 4 cardinal neighbors of the cell
f.	get8neighbors() – returns all 8 neighbors(cardinal + diagonal) of the cell

7.	Interface.py -> creates a GUI interface based on the data obtained from the config file

8.	Simulator.py -> Main function that simulates the algorithms on the given map.

9.	BFS.py , Dijsktra.py, Astar.py -> The working algorithms which work the simulator
