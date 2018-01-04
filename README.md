# Path-Planning-Simulator
Python based simulator for path planning algorithms such as A*, JPS, Dijkstra, etc

<p>The project builds upon the <a href="http://ieeexplore.ieee.org/document/7379179/"> CAC </a> simulator implemented by Prof.Avinash to make it faster and more efficient. The bottle neck, as found by profiling the code was turning out to be the path planning algorithm for the simulator. To tackle this issue, comparative analysis of state-of-the-art path planning and exploration algorithms via simulation and experimentation was performed in this project. The simulation is performed on 50+ custom grid maps of size 50 x 50 on a python based simulator. Experiments are also performed on a Turtlebot, on which a node containing all studied path planning algorithms is written on the ROS Indigo platform and tests on 10+ real maps are performed (On Gazebo/Rviz as well as in the lab). The following graph search algorithms were tested in the study:</p>

<ol>
<li>Breadth First Search</li>
<li> Depth First Search</li>
<li>Dijkstra's Algorithm</li>
<li>A-Star Algorithm</li>
<li>D-Star Algorithm</li>
<li>Greedy Best First Search</li>
<li>Jump Point Search (JPS)</li>
<li>Jump Point Search extensions - JPS+ and JPS+B</li>
</ol>

<p>JPS was found to be the best performing algorithm on the simulator as well as the Turtlebot.</p>
<iframe src="https://github.com/sahibdhanjal/Path-Planning-Simulator/blob/master/JavaScript%20Simulator/Path%20Planners.html" name="targetframe" allowTransparency="true" scrolling="no" frameborder="0" ></iframe>
