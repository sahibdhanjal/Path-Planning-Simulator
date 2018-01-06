# Path Planning Simulator
This folder includes the Javasript based simulator. The following algorithms are being simulated in this implementation:
1. Breadth First Search (BFS)
2. Depth First Search (DFS)
3. Greedy Best First Search
4. Dijkstra's Algorithm
5. A-Star Algorithm (A*)
6. Rapidly Exploring Random Trees (RRT)
7. RRT - Connect
8. RRT - Star

To use the simulator, clone the repository to your computer and launch index.html which contains the web interface. The source file is located in the scripts folder. Upon loading, the site interface looks as follows, showing the algorithms on top with a textbox for statistics:

<p align="center">
  <img src= images/interface.png width = 400px>
  <img src= images/bfs.png width = 400px>
</p>

The respective planners can be invoked by clicking on the appropriate buttons. The textbox shows relevant statistics like the queued nodes, planner name, start and goal, when the planner is running. The preview of BFS, A*, and RRT-Connect is shown below

<p align="center">

  <img src= images/astar.png width = 400px>
  <img src= images/rrtconnect.png width = 400px>
</p>


The start and goal nodes can be changed by modifying the URL as follows:

../index.html ? q_init = [3,3] ? q_goal = [4.5,4.5]
