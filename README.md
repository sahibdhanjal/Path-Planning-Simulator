# Path-Planning-Simulator
The repository contains two simulators -
## Python Simulator
### Path Planning Algorithms
A python based simulator for path planning algorithms such as A*, JPS, Dijkstra, and other state-of-the-art algorithms.
<p>The project builds upon the <a href="http://ieeexplore.ieee.org/document/7379179/"> Cluster Allocate Cover (CAC) </a> simulator implemented by Prof.Avinash to make it faster and more efficient. The bottle neck, as found by profiling the code was turning out to be the path planning algorithm for the simulator. To tackle this issue, comparative analysis of state-of-the-art path planning and exploration algorithms via simulation and experimentation was performed in this project. The simulation is performed on 50+ custom grid maps of size 50 x 50 on a python based simulator. Experiments are also performed on a Turtlebot, on which a node containing all studied path planning algorithms is written on the ROS Indigo platform and tests on 10+ real maps are performed (On Gazebo/Rviz as well as in the lab). The following graph search algorithms were tested in the study:</p>
<ol>
<li>Breadth First Search</li>
<li>Depth First Search</li>
<li>Dijkstra's Algorithm</li>
<li>A-Star Algorithm</li>
<li>D-Star Algorithm</li>
<li>Greedy Best First Search</li>
<li>Jump Point Search (JPS)</li>
<li>Jump Point Search extensions - JPS+ and JPS+B</li>
</ol>
<p>JPS was found to be the best performing algorithm on the simulator as well as the Turtlebot.</p>

### Exploration Algorithms
The srepository also includes implementations of the following frontier based exploration algorithms
<ol>
  <li>Yamauchi's Frontier Algorithm</li>
  <li>Burgard's Frontier Algorithm</li>
  <li>Faigl's Frontier Algorithm</li>
  <li>Cluster Allocate Cover</li>
</ol>

## Javascript Simulator
A simple extension of the simulator has been implemented in JavaScript for ease of viewing in web browsers. The JavaScript version contains the following algorithms:
<ol>
<li>Breadth First Search</li>
<li>Depth First Search</li>
<li>Dijkstra's Algorithm</li>
<li>Greedy Best First Search</li>
<li>A-Star Algorithm</li>
<li>Rapidly Exploring Random Trees (RRT)</li>
<li>RRT-Connect</li>
<li>RRT-Star</li>
</ol>
Just one map has been created for this simulator, this being the secondary version to the Python Simluator.

## TurtleBot Path Planning Node
A path planning node for Robot Operating System (ROS-Indigo) has been implemented and can be found in the /Turtlebot Node - Path Planning/ folder of this repository. To integrate this node into your ROS package, you may follow <a href="http://www.iroboapp.org/index.php?title=Adding_Relaxed_Astar_Global_Path_Planner_As_Plugin_in_ROS">this</a> article.
This package contains implementations of the following algorithms:
  1. Depth First Search
  2. Breadth First Search
  3. Greedy Best First Search
  4. Dijkstra's Algorithm
  5. A-Star Algorithm
  6. D-Star Algorithm
  7. Jump Point Search
  
  
