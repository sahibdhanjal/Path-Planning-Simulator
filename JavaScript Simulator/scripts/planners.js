//////////////////////////////////////////////////
/////     MAIN LOOP
//////////////////////////////////////////////////


function init(algo) {
	search_alg="";

	// get user input as to which algorith to use
	switch(algo){
		case 0:
			search_alg = "depth-first";
			break;
		case 1:
			search_alg = "breadth-first";
			break;
		case 2:
			search_alg = "greedy-best-first";
			break;
		case 3:
			search_alg = "dijkstra";
			break;
		case 4:
			search_alg = "A-star";
			break;
		case 5:
			search_alg = "RRT";
			break;
		case 6:
			search_alg = "RRT-connect";
			break;
		case 7:
			search_alg = "RRT-star";
			break;
	}

    initSearch(algo);
    animate();
}


//////////////////////////////////////////////////
/////     SEARCH INITIALIZATION LOOP
//////////////////////////////////////////////////


function initSearch() {
	// World defined by self
    planning_scene = "multi_part";
    
    // eps defines the density of the grid cells
    eps = 0.1;
    path = [];

    // create event handlers for the mouse
    canvas = document.getElementById("myCanvas");
    mouse_x = 0;
    mouse_y = 0;

    // when the mouse moves, update the mouse's location
    canvas.onmousemove = function handleMouseMove(event) {
        mouse_x = event.clientX;
        mouse_y = event.clientY;
    };

    // when the mouse button is pressed, update mouseDown
    canvas.onmousedown = function() { 
        mouseDown = 1; 
    };

    // when the mouse button is released, update mouseDown
    canvas.onmouseup = function() {
        mouseDown = 0;
    };

    // Set the Start and Goal Points on the Canvas
    q_init = [-1.2,-1.2];
    q_goal = [5.3, 3.5];

        // Fixed code for parsing full url to get input parameters as given
    var url_parsed = window.location.href.split("?");
    for (i=1;i<url_parsed.length;i++) {
        var param_parsed = url_parsed[i].split("=");
        // console.log(param_parsed[0],param_parsed[1]);
        // eval(param_parsed[0]+"=\'"+param_parsed[1]+"\'");
        var param = param_parsed[0] ; var arg = param_parsed[1];
        switch(param) {
            case "q_init":
                arg = arg.slice(1,arg.length-1);
                var args = arg.split(",");
                q_init[0] = parseFloat(args[0]) ; q_init[1] = parseFloat(args[1])
                break;
            case "q_goal":
                arg = arg.slice(1,arg.length-1);
                var args = arg.split(",");
                q_goal[0] = parseFloat(args[0]) ; q_goal[1] = parseFloat(args[1])
                break;
            default:
                console.warn("Using default parameters");
                q_init = [-1.2,-1.2];
                q_goal = [5.3, 3.5];
                search_alg = "A-star";
        }
    }
    
    // Convert the Canvas Coordinates to Grid Coordinates
    start = getCoord(q_init);
    goal = getCoord(q_goal);

    // set the world for the planner 
    setPlanningScene();

    // initialize search tree from start configurations (RRT-based algorithms)
    T_a = initRRT(q_init, "T_a");

    // also initialize search tree from goal configuration (RRT-Connect)
    T_b = initRRT(q_goal, "T_b");

    // variables for RRT
    nbhd = 2;
    stepSize = 1*eps;
    tree1 = T_a;
    tree2 = T_b;
    q_new = [];
    target = [];
    tolerance = 2*stepSize;
    
    saved_iter = 0;
    
	// Initialize everything to zero    
    initSearchGraph();
    search_iterate = true;
    search_iter_count = 0;
    search_result = "starting";
    search_max_iterations = 100000;
    search_visited = 0;
    path_length = 0;
    path_found = false;
    cur_time = Date.now();
    min_msec_between_iterations = 20;
    
    // Add TextBar Stats Element
    textbar = document.getElementById("textbar");
}



//////////////////////////////////////////////////
/////     ANIMATION AND INTERACTION LOOP
//////////////////////////////////////////////////

function animate() {
    drawRobotWorld();

    // make sure the rrt iterations are not running faster than animation update
    if (search_iterate && (Date.now()-cur_time > min_msec_between_iterations)) {
        cur_time = Date.now();
        search_iter_count++; 

        switch (search_alg) {
            case "depth-first":
                search_result = DFS();
                break;
            case "breadth-first": 
                search_result = BFS();
                break;
            case "dijkstra":
                search_result = Dijkstra();
                break;
            case "greedy-best-first": 
                search_result = Greedy();
                break;
            case "A-star": 
                search_result = iterateGraphSearch();
                break;
            case "RRT": 
                search_result = iterateRRT();
                break;
            case "RRT-connect": 
                search_result = iterateRRTConnect();
                break;
            case "RRT-star": 
                search_result = iterateRRTStar();
                break;
            default: 
                console.warn('search_canvas: search algorithm not found, using rrt as default');
                search_result = iterateRRT();
                break;
        }
    }

    var queue_size;
    if(search_alg=="depth-first" || search_alg=="breadth-first" || search_alg=="RRT" || search_alg=="RRT-connect" || search_alg == "RRT-star")
        queue_size = visit_queue.length;
    else
        queue_size = visit_queue.size();

    if(search_alg=="A-star" || search_alg=="depth-first" || search_alg=="breadth-first" || search_alg=="greedy-best-first" || search_alg=="dijkstra") {
        textbar.innerHTML = 
        "<h3>Algorithm Statistics:</h3>"
        + search_alg
        + " progress: " + search_result
        + "<br>"
        + "<strong>Start</strong>: " + q_init
        + "  |  "
        + "<strong>Target</strong>: " + q_goal
        + "<br>"
        + "<strong>Iteration:</strong> " + search_iter_count
        + "  |  "
        + "<strong>Path Length:</strong> " + path_length.toFixed(2)
        + "<br>"
        + "<strong>Visited:</strong> " + search_visited
        + "   |   "
        + "<strong>Queued:</strong> " + queue_size
        + "<br>" ;
        
        //textbar.innerHTML += "<br> mouse ("+ mouse_x+","+mouse_y+")";
    }

    else if(search_alg=="RRT" || search_alg=="RRT-connect"){
        textbar.innerHTML = 
        "<h3>Algorithm Statistics:</h3>"
        + search_alg 
        + " progress: " + search_result
        + " <br> "
        + "<strong>Start</strong>: " + q_init
        + "  |  "
        + "<strong>Target</strong>: " + q_goal
        + "<br>"
        + "<strong>Iteration:</strong> " + search_iter_count
        + "  |  "
        + "<strong>Path Length:</strong> " + path_length.toFixed(2)
        + "<br>";
    }

    else{
        textbar.innerHTML =
        "<h3>Algorithm Statistics:</h3>"
        + search_alg 
        + " progress: " + search_result
        + " <br> "
        + "<strong>Start</strong>: " + q_init
        + "  |  "
        + "<strong>Target</strong>: " + q_goal
        + "<br>"
        + "<strong>Iteration:</strong> " + search_iter_count
        + "  |  "
        + "<strong>Path Length:</strong> " + path_length.toFixed(2)
        + "<br>"
        + "<strong>Optimization</strong> | 2000 Iterations: " + path_found
        + "<br>";
    }

    textbar.innerHTML += "<br> Mouse Position: ("+ xformViewWorldX(mouse_x)+","+xformViewWorldY(mouse_y)+")";

    // callback request for the animate function be called again
    //   more details online:  http://learningwebgl.com/blog/?p=3189
    if(search_result=="succeeded" || search_iter_count>search_max_iterations)
        search_iterate =false;
    
    requestAnimationFrame(animate);
}

function BFS() {
    if(visit_queue.length == 0 || G[goal[0]][goal[1]].obstacle==true || G[start[0]][start[1]].obstacle==true) {
        console.log("Search Failed!")
        search_iterate=false;
        return "failed";
    }

    curr = visit_queue.shift();
    var xc = curr[0] ; var yc = curr[1];
    G[xc][yc].queued = false;
    ctx.fillStyle = "#ffe8a5";
    ctx.fillRect(xformWorldViewX(G[xc][yc].x)-3,xformWorldViewY(G[xc][yc].y)-3,6,6);

    if(xc == goal[0] && yc==goal[1] || G[goal[0]][goal[1]].visited==true) {
        console.log("Goal found!");
        search_iterate=false;
        draw_path();
        return "succeeded";
    }

    var neighbors = get4nbhd(curr);
    for(var i=0; i<neighbors.length; i++) {
        var xi = neighbors[i][0]; var yi = neighbors[i][1];
        if(G[xi][yi].visited==false) {
            visit_queue.push([xi,yi]);
            G[xi][yi].visited = true;
            G[xi][yi].queued = true;
            G[xi][yi].parent = curr;
            G[xi][yi].distance = G[xc][yc].distance + gcost(curr,[xi,yi]);
            search_visited+=1;
            ctx.fillStyle = "#52483F";
            ctx.fillRect(xformWorldViewX(G[xi][yi].x)-3,xformWorldViewY(G[xi][yi].y)-3,6,6);
            // console.log(curr,next,G[xi][yi].distance,G[xi][yi].visited, G[xi][yi].priority,G[xi][yi].queued,G[xi][yi].parent,search_visited)
        }
    }
    return "iterating";
}

function DFS() {
    if(visit_queue.length == 0 || G[goal[0]][goal[1]].obstacle==true || G[start[0]][start[1]].obstacle==true) {
        console.log("Search Failed!")
        search_iterate=false;
        return "failed";
    }

    curr = visit_queue.pop();
    var xc = curr[0] ; var yc = curr[1];
    G[xc][yc].queued = false;
    ctx.fillStyle = "#ffe8a5";
    ctx.fillRect(xformWorldViewX(G[xc][yc].x)-3,xformWorldViewY(G[xc][yc].y)-3,6,6);
    
    if((curr[0] == goal[0] && curr[1]==goal[1]) || G[goal[0]][goal[1]].visited==true) {
        console.log("Goal found!");
        search_iterate=false;
        draw_path();
        return "succeeded";
    }

    var neighbors = get4nbhd(curr);
    for(var i=0; i<neighbors.length; i++) {
        var xi = neighbors[i][0]; var yi = neighbors[i][1];
        if(G[xi][yi].visited==false) {
            visit_queue.push([xi,yi]);
            G[xi][yi].visited = true;
            G[xi][yi].queued = true;
            G[xi][yi].parent = curr;
            G[xi][yi].distance = G[xc][yc].distance + gcost(curr,[xi,yi]);
            search_visited+=1;
            ctx.fillStyle = "#52483F";
            ctx.fillRect(xformWorldViewX(G[xi][yi].x)-3,xformWorldViewY(G[xi][yi].y)-3,6,6);
        }
    }
    return "iterating";
}

function Dijkstra() {
    if(visit_queue.size()==0 || G[goal[0]][goal[1]].obstacle==true || G[start[0]][start[1]].obstacle==true) {
        console.log("Search failed");
        search_iterate=false;
        return "failed";
    }

    curr = visit_queue.get();
    var xc=curr[0] ; var yc=curr[1];
    G[xc][yc].queued = false;
    ctx.fillStyle = "#ffe8a5";
    ctx.fillRect(xformWorldViewX(G[xc][yc].x)-3,xformWorldViewY(G[xc][yc].y)-3,6,6);

    if(xc == goal[0] && yc==goal[1] || G[goal[0]][goal[1]].visited==true) {
        console.log("Goal found!");
        search_iterate=false;
        draw_path();
        return "succeeded";
    }

    var nbrs = get4nbhd(curr);
    for(var i=0 ; i<nbrs.length;i++) {
        var next = nbrs[i];
        var xi=next[0]; var yi=next[1];
        var new_cost = G[xc][yc].distance + gcost(curr,next);
        

        if(G[xi][yi].visited==false || new_cost < G[xi][yi].distance) {
            G[xi][yi].distance = new_cost;
            var pri = new_cost;
            visit_queue.put(next,pri);
            G[xi][yi].priority = pri;
            G[xi][yi].visited = true;
            G[xi][yi].queued = true;
            G[xi][yi].parent = curr;
            search_visited+=1;
            ctx.fillStyle = "#52483F";
            ctx.fillRect(xformWorldViewX(G[xi][yi].x)-3,xformWorldViewY(G[xi][yi].y)-3,6,6);

        }
    }
    return "iterating";
}

function Greedy() {
    if(visit_queue.size()==0 || G[goal[0]][goal[1]].obstacle==true || G[start[0]][start[1]].obstacle==true) {
        console.log("Search failed");
        search_iterate=false;
        return "failed";
    }

    curr = visit_queue.get();
    var xc=curr[0] ; var yc=curr[1];
    G[xc][yc].queued = false;
    ctx.fillStyle = "#ffe8a5";
    ctx.fillRect(xformWorldViewX(G[xc][yc].x)-3,xformWorldViewY(G[xc][yc].y)-3,6,6);

    if(xc==goal[0] && yc==goal[1] || G[goal[0]][goal[1]].visited==true) {
        console.log("Goal found!");
        search_iterate=false;
        draw_path();
        return "succeeded";
    }

    var nbrs = get4nbhd(curr);
    for(var i=0 ; i<nbrs.length;i++) {
        var next = nbrs[i];
        var xi=next[0]; var yi=next[1];
        var new_cost = heuristic(next,goal);

        if(G[xi][yi].visited==false || new_cost < G[xi][yi].distance) {
            G[xi][yi].distance = new_cost;
            var pri = new_cost;
            visit_queue.put(next,pri);
            G[xi][yi].priority = pri;
            G[xi][yi].visited = true;
            G[xi][yi].queued = true;
            G[xi][yi].parent = curr;
            search_visited+=1;
            ctx.fillStyle = "#52483F";
            ctx.fillRect(xformWorldViewX(G[xi][yi].x)-3,xformWorldViewY(G[xi][yi].y)-3,6,6);
        }
    }
    return "iterating";
}

function iterateGraphSearch() {
    if(visit_queue.size()==0 || G[goal[0]][goal[1]].obstacle==true || G[start[0]][start[1]].obstacle==true) {
        console.log("Search failed");
        search_iterate=false;
        return "failed";
    }

    curr = visit_queue.get();
    var xc=curr[0] ; var yc=curr[1];
    G[xc][yc].queued = false;
    ctx.fillStyle = "#ffe8a5";
    ctx.fillRect(xformWorldViewX(G[xc][yc].x)-3,xformWorldViewY(G[xc][yc].y)-3,6,6);

    if(xc==goal[0] && yc==goal[1] || G[goal[0]][goal[1]].visited==true) {
        console.log("Goal found!");
        search_iterate=false;
        draw_path();
        return "succeeded";
    }

    var nbrs = get4nbhd(curr);
    for(var i=0 ; i<nbrs.length;i++) {
        var next = nbrs[i];
        var xi=next[0]; var yi=next[1];
        var new_cost = G[xc][yc].distance + gcost(curr,next);

        if(G[xi][yi].visited==false || new_cost < G[xi][yi].distance) {
            G[xi][yi].distance = new_cost;
            var pri = new_cost + heuristic(goal,next);
            visit_queue.put(next,pri);
            G[xi][yi].priority = pri;
            G[xi][yi].visited = true;
            G[xi][yi].queued = true;
            G[xi][yi].parent = curr;
            search_visited+=1;
            ctx.fillStyle = "#52483F";
            ctx.fillRect(xformWorldViewX(G[xi][yi].x)-3,xformWorldViewY(G[xi][yi].y)-3,6,6);
        }
    }
    return "iterating";
}

function iterateRRT() {
    var rand = randomConfig();

    if(testCollision(q_goal)==true || testCollision(q_init)==true || search_iter_count>=search_max_iterations) {
        console.log("Search Failed!");
        search_iterate=false;
        return "failed";
    }

    var result = extendRRT(rand,q_goal,T_a);
    if (result=="reached"){
        console.log("Reached target!");
        dfsPath(q_goal, q_init, T_a);
        return "succeeded";
    }
    return "iterating";
}

function iterateRRTConnect() {
    if(testCollision(q_goal)==true || testCollision(q_init)==true || search_iter_count>=search_max_iterations) {
        console.log("Search Failed!");
        search_iterate=false;
        return "failed";
    }

    var rand = randomConfig();
    if(extendRRT(rand, target, tree1)!="trapped")
        if(connectRRT(q_new,tree2)=="reached"){
            console.log("reached");
            dfsPath(q_new, q_init, T_a);
            dfsPath(q_new,q_goal,T_b);
            search_iterate=false;
            return "succeeded";
        }
    
    if(tree1.name == "T_a"){
        tree1 = T_b;
        tree2 = T_a;
        target = q_goal;
    }
    else{
        tree1 = T_a;
        tree2 = T_b;
        target = q_init;
    }

    return "iterating";
}

function iterateRRTStar() {
    if(testCollision(q_goal)==true || testCollision(q_init)==true || search_iter_count>=search_max_iterations) {
        console.log("Search Failed!");
        search_iterate=false;
        return "failed";
    }

    var rand = randomConfig(),
        nearest = findNearestNeighbor(rand,T_a),
        vertex = newConfig(nearest, rand, T_a);

    if(distance(vertex, q_goal) <= 1.4*stepSize)
        vertex = q_goal;

    if(testCollision(vertex)==false && inTree(vertex, T_a)==false){
        var near = findNeighborhood(vertex,T_a),
            parent_idx = chooseParent(vertex, nearest, near, T_a),
            parent = T_a.vertices[parent_idx].vertex,
            cost = T_a.vertices[parent_idx].cost + distance(parent,vertex);

        insertTreeVertex(T_a, vertex, parent, cost);
        insertTreeEdge(T_a, T_a.newest ,parent_idx);
        reWire(vertex, nearest, near, T_a);

        q_new = vertex;

        if(isEqual(vertex, q_goal)==true){
            saved_iter = search_iter_count;
            path_found = true;
            console.log("Reached target!");
            return "reached";
        }
    }
    var goal_idx = getIndex(q_goal,T_a);
    
    if (saved_iter>0) {
        if(search_iter_count - saved_iter>2000){
            search_iterate = false;
            dfsPath(q_goal, q_init, T_a);
            return "reached";
        }
        return "reached target! Optimizing";
    }

    else
        return "iterating";
}


//////////////////////////////////////////////////
/////     RRT IMPLEMENTATION FUNCTIONS
//////////////////////////////////////////////////

function extendRRT(rand, target, tree) {
    var nnbr_idx = findNearestNeighbor(rand, tree),
        vertex = newConfig(nnbr_idx, rand, tree),
        parent = tree.vertices[nnbr_idx].vertex,
        cost = tree.vertices[nnbr_idx].cost + distance(vertex,parent);

    if (distance(vertex, target) <= 1.2*stepSize)
        vertex = target;
    
    if (testCollision(vertex)==false && inTree(vertex,tree)==false){
        insertTreeVertex(tree,vertex,parent, cost);
        insertTreeEdge(tree,tree.newest,nnbr_idx);
        
        q_new = vertex;

        if (isEqual(vertex, target)==true){
            search_iterate = false;
            return "reached";
        }
        
        return "advanced";
    }

    else
        return "trapped";
}

function connectRRT(target, tree) {
    var result = extendRRT(target, target, tree);
    if (result=="advanced")
        result = extendRRT(target, target, tree);
    return result;
}

// Returns a random config in the C-Space
function randomConfig() {
    var x = strip(-1.5 + Math.random()*7.5),
        y = strip(-1.5 + Math.random()*7.5);
    return [x,y];
}

// Returns the new config to be added to the tree whether or not valid
function newConfig(q_idx, rand, tree) {
    var dx = rand[0] - tree.vertices[q_idx].vertex[0],
        dy = rand[1] - tree.vertices[q_idx].vertex[1],
        n = Math.sqrt(dx*dx + dy*dy),
        
        xd = tree.vertices[q_idx].vertex[0] + stepSize*dx/n,
        yd = tree.vertices[q_idx].vertex[1] + stepSize*dy/n;

    xd = Math.round(xd*100)/100;
    yd = Math.round(yd*100)/100;

    return [xd,yd];
}

// Returns the distance between 2 vectors
function distance(a,b) {
    var dx = a[0]-b[0],
        dy = a[1]-b[1],
        dist = Math.sqrt(dx*dx + dy*dy);
    return dist;
}

// Returns the index of the nearest neighbour
function findNearestNeighbor(target, tree) {
    var min = 999999,
        pos = 0,
        dist = 0;

    for(var i=0; i<=tree.newest; i++) {
        var dist = distance(target,tree.vertices[i].vertex);
        if (min >= dist) {min = dist; pos = i;}
    }
    return pos;
}

// Returns all node indexes in neighborhood
function findNeighborhood(target, tree) {
    var nbrs = [];
    for(var i=0; i<=tree.newest; i++)
        if (distance(target,tree.vertices[i].vertex) <= tolerance)
            nbrs.push(i);
    return nbrs;
}

// Chooses parent with minimum cost
function chooseParent(target, min_idx, nbr_idxs, tree) {
    var cmin = tree.vertices[min_idx].cost + stepSize,
        pos = min_idx;

    if(nbr_idxs.length>0){
        for(var i=0; i<nbr_idxs.length;i++){
            var cost = tree.vertices[nbr_idxs[i]].cost + distance(tree.vertices[nbr_idxs[i]].vertex, target);
            if (cost<cmin)
                pos = nbr_idxs[i];
        }
    }
    return pos;
}

// Rewires the structure
function reWire(target, min_idx, nbr_idxs, tree) {
    if(nbr_idxs.length>0)
        for(var i=0; i<nbr_idxs.length; i++){
            var cost = tree.vertices[nbr_idxs[i]].cost;
            var new_cost = tree.vertices[tree.newest].cost + distance(tree.vertices[tree.newest].vertex, tree.vertices[nbr_idxs[i]].vertex);
            if (new_cost<cost){
                tree.vertices[nbr_idxs[i]].cost = new_cost;
                var parent_idx = getIndex(tree.vertices[nbr_idxs[i]].parent,tree);
                tree.vertices[nbr_idxs[i]].parent = tree.vertices[tree.newest].vertex;
                insertTreeEdge(tree,nbr_idxs[i],tree.newest);
                removeTreeEdge(tree, parent_idx, nbr_idxs[i]);
            }
        }
}

// Generates path between the current "node" to the "target" in a given tree
function dfsPath(node, target, tree) {
    var curr = node,
        path = [],
        node_idx = getIndex(node,tree);

    while(isEqual(curr,target)==false){
        path.push(curr);
        var curr_idx = getIndex(curr,tree),
            parent = tree.vertices[curr_idx].parent;
        draw_2D_path_configurations(curr,parent);
        path_length+=distance(curr,parent);
        curr = parent;
    }
}

// Prints the elements of an array
function print(arr) {
    var st = "Path := \n";
    
    for(var i=0;i<arr.length;i++)
        st += "["+arr[i] + "]\n"
    console.log(st);
}

// Checks if the vertex is already in the tree or not
function inTree(vertex, tree) {
    for(var i=0; i<=tree.newest; i++)
        if (isEqual(vertex, tree.vertices[i].vertex))
            return true;
    return false;
}

// Checks if arrays are equal
function isEqual(q1,q2) {
    for(var i=0; i<q1.length; i++)
        if (q1[i]!=q2[i])
            return false;
    return true;
}

// Returns index of vertex, if it's in the tree
function getIndex(vertex, tree) {
    if (inTree(vertex,tree))
        for(var i=0; i<=tree.newest; i++)
            if (isEqual(vertex, tree.vertices[i].vertex))
                return i;
    return null;
}

//////////////////////////////////////////////////
/////     STENCIL SUPPORT FUNCTIONS
//////////////////////////////////////////////////

// functions for transforming canvas coordinates into planning world coordinates
function xformWorldViewX(world_x) {
    return (world_x*100)+200;  // view_x
}
function xformWorldViewY(world_y) {
    return (world_y*100)+200;  // view_y
}
function xformViewWorldX(view_x) {
    return (view_x-200)/100;  // view_x
}
function xformViewWorldY(view_y) {
    return (view_y-200)/100;  // view_y
}

function drawRobotWorld() {
    // draw start and goal configurations
    c = document.getElementById("myCanvas");
    ctx = c.getContext("2d");
    ctx.fillStyle = "red";
    ctx.fillRect(xformWorldViewX(q_init[0])-5,xformWorldViewY(q_init[1])-5,12,12);
    ctx.fillStyle = "green";
    ctx.fillRect(xformWorldViewX(q_goal[0])-5,xformWorldViewY(q_goal[1])-5,12,12);

    // draw robot's world
    for (j=0;j<range.length;j++) { 
        ctx.fillStyle = "#e4e4e4";
        ctx.fillRect(xformWorldViewX(range[j][0][0]),xformWorldViewY(range[j][1][0]),xformWorldViewX(range[j][0][1])-xformWorldViewX(range[j][0][0]),xformWorldViewY(range[j][1][1])-xformWorldViewY(range[j][1][0]));
    }
}

function initSearchGraph() {

    // initialize search graph as 2D array over configuration space 
    //   of 2D locations with specified spatial resolution 
    G = [];
    for (iind=0,xpos=-2;xpos<7;iind++,xpos+=eps) {
        G[iind] = [];
        for (jind=0,ypos=-2;ypos<7;jind++,ypos+=eps) {
            G[iind][jind] = {
                i:iind,j:jind, // mapping to graph array
                x:strip(xpos),y:strip(ypos), // mapping to map coordinates
                parent:null, // pointer to parent in graph along motion path
                distance:10000, // distance to start via path through parent
                visited:false, // flag for whether the node has been visited
                priority:null, // visit priority based on fscore
                queued:false, // flag for whether the node has been queued for visiting
                obstacle:false  // flag to determine whether node is obstacle
            };
        }
    }

    define_obstacles();

    // For algorithms using priority queues as data structures - A-Star/ Dijkstra/ Greedy
    if(search_alg=="dijkstra" || search_alg=="A-star" || search_alg=="greedy-best-first") {
        visit_queue = new PriorityQueue();
        visit_queue.put(start,0);
        G[start[0]][start[1]].visited = true;
        G[start[0]][start[1]].distance = 0;
        G[start[0]][start[1]].queued = true;
        G[start[0]][start[1]].priority = 0;
    }
    // else if(search_alg == "RRT" || search_alg == "RRT-connect" || search_alg == "RRT-star") {
    //     visit_queue = [];
    // }
    // For algorithms using stacks/queues as data structures - DFS/ BFS
    else {
        visit_queue = [];
        visit_queue.push(start);
        G[start[0]][start[1]].visited = true;
        G[start[0]][start[1]].queued = true;
    }

    rows = G.length; cols = G[0].length;
}

function setPlanningScene() {

    // obstacles specified as a range along [0] (x-dimension) and [1] y-dimension
    range = []; // global variable

    // world boundary
    range[0] = [ [-1.9,7.9],[-1.9,-1.7] ];
    range[1] = [ [-1.9,7.9],[5.7,5.9] ];
    range[2] = [ [-1.9,-1.7], [-1.9,5.9] ];
    range[3] = [ [7.7,7.9],   [-1.9,5.9] ];

    if (typeof planning_scene === 'undefined')
        planning_scene = 'multi_part';

	if (planning_scene == 'multi_part') {
        range[4] = [ [0.2,0.4], [-1.9,-1.5]];
        range[5] = [ [0.2,0.4], [-1.2, 4.1]];
        range[6] = [ [0.2,0.4],[4.4,5.9]];
        
        range[7] = [ [1.8,2],[-1.9,0.8]];
        range[8] = [ [1.8,2],[1.1,5.4]];
        
        range[9] = [ [3.6,6.5],[0.6,0.8]];
        range[10] = [ [3.6,6.5],[4.9,5.1]];
        range[11] = [ [3.6,3.8],[1.1,4.6]];
        range[12] = [ [6.3,6.5],[1.1,4.6]];

        range[13] = [ [3.6,4.5],[2.6,2.8]];
        range[14] = [ [4.9,6.5],[2.6,2.8]];
    }
}

function testCollision(q) {
    var j;
    for (j=0;j<range.length;j++) { 
        var in_collision = true; 
        for (i=0;i<q.length;i++) { 
            if ((q[i]<range[j][i][0])||(q[i]>range[j][i][1]))
                in_collision = false;
        }
        if (in_collision)
            return true;
    }
    return false;
}

function initRRT(q,name) {
    var tree = {};

    tree.vertices = [];
    tree.vertices[0] = {};
    tree.vertices[0].vertex = q;
    tree.vertices[0].edges = [];
    tree.vertices[0].parent = null;
    tree.vertices[0].cost = 0;
    tree.name = name;

    tree.newest = 0;

    return tree;
}

function insertTreeVertex(tree, q, parent, cost) {
    new_vertex = {};
    new_vertex.edges = [];
    new_vertex.vertex = q;
    new_vertex.parent = parent;
    new_vertex.cost = cost;

    tree.vertices.push(new_vertex);
    tree.newest = tree.vertices.length - 1;
    draw_2D_configuration(q);
}

function draw_2D_configuration(q) {
    c = document.getElementById("myCanvas");
    ctx = c.getContext("2d");
    ctx.fillStyle = "#80685E";
    ctx.fillRect(xformWorldViewX(q[0])-3,xformWorldViewY(q[1])-3,6,6);
}

function draw_2D_edge_configurations(q1,q2) {
    c = document.getElementById("myCanvas");
    ctx = c.getContext("2d");
    ctx.beginPath();
    ctx.moveTo(xformWorldViewX(q1[0]),xformWorldViewY(q1[1]));
    ctx.lineTo(xformWorldViewX(q2[0]),xformWorldViewY(q2[1]));
    ctx.strokeStyle = "#2a2e2f"; 
    ctx.stroke();
}

function remove_2D_edge_configurations(q1,q2) {
    c = document.getElementById("myCanvas");
    ctx = c.getContext("2d");
    ctx.beginPath();
    ctx.moveTo(xformWorldViewX(q1[0]),xformWorldViewY(q1[1]));
    ctx.lineTo(xformWorldViewX(q2[0]),xformWorldViewY(q2[1]));
    ctx.strokeStyle = "#DC143C"; 
    // ctx.lineWidth = 2;
    ctx.stroke();
}

function draw_2D_path_configurations(q1,q2) {
    c = document.getElementById("myCanvas");
    ctx = c.getContext("2d");
    ctx.beginPath();
    ctx.moveTo(xformWorldViewX(q1[0]),xformWorldViewY(q1[1]));
    ctx.lineTo(xformWorldViewX(q2[0]),xformWorldViewY(q2[1]));
    ctx.strokeStyle = "#283747";
    ctx.lineWidth = 4;
    ctx.stroke();
}

function insertTreeEdge(tree,q1_idx,q2_idx) {
    tree.vertices[q1_idx].edges.push(tree.vertices[q2_idx]);
    tree.vertices[q2_idx].edges.push(tree.vertices[q1_idx]);
    draw_2D_edge_configurations(tree.vertices[q1_idx].vertex,tree.vertices[q2_idx].vertex); 
}

function removeTreeEdge(tree, q1_idx, q2_idx) {
    var edge1 = tree.vertices[q1_idx].edges,
        edge2 = tree.vertices[q2_idx].edges,
        vertex1 = tree.vertices[q1_idx].vertex,
        vertex2 = tree.vertices[q2_idx].vertex,
        l1 = edge1.length,
        l2 = edge2.length,
        pos = 0;

    for(var i=0; i<l1; i++)
        if (isEqual(edge1[i],vertex2))
            pos = i;
    edge1.splice(pos,1);

    for(var i=0; i<l2; i++)
        if (isEqual(edge2[i],vertex1))
            pos = i;
    edge2.splice(pos,1);

    remove_2D_edge_configurations(vertex1,vertex2);
}

//////////////////////////////////////////////////
/////     PRIORITY QUEUE IMPLEMENTATION
//////////////////////////////////////////////////

// STENCIL: implement min heap functions for graph search priority queue.
//   These functions work use the 'priority' field for elements in graph.
function PriorityQueue() {this.list = [];}

PriorityQueue.prototype.put = function(element, priority) {
  for (var i = 0; i < this.list.length && this.list[i][1] < priority; i++);
  this.list.splice(i, 0, [element, priority])}

PriorityQueue.prototype.get = function() {return this.list.shift()[0]}

PriorityQueue.prototype.size = function() {return this.list.length}  

PriorityQueue.prototype.qprint = function() {
    var st="Queue:";
    for(var i=0;i<this.list.length;i++)
        st+= " | "+this.list[i][0];
    console.log(st)}

//////////////////////////////////////////////////
/////     HELPER FUNCTIONS
//////////////////////////////////////////////////

// Function to define all obstacles in the canvas frame
function define_obstacles() {
    for(var i= 0; i<G.length; i++) {
        for(var j=0;j<G[0].length;j++) {
            var x = G[i][j].x; var y = G[i][j].y
            if(testCollision([x,y]) == true)
                G[i][j].obstacle = true;
        }
    }
}
// Function to get graph coordinates[(0,90),(0,90)] by giving world coordinates [(-2,7),(-2,7)] as inputs
function getCoord(node) {
    var x = Math.round(strip((node[0]+2)/eps));
    var y = Math.round(strip((node[1]+2)/eps));
    return [x,y]
}
// Function to get world coordinates[(-2,7),(-2,7)] by giving graph coordinates [(0,90),(0,90)] as inputs
function getWorld(node) {
    var x = strip(eps*node[0]) - 2;
    var y = strip(eps*node[1]) - 2;
    return [x,y]
}
// Function to get 4 adjacent neighbours
function get4nbhd(node) {
    var nbhd = [];
    var space = [[0,1],[1,0],[0,-1],[-1,0]];
    for(var i=0; i< space.length; i++) {
        var new_x = node[0] + space[i][0];
        var new_y = node[1] + space[i][1];
        if((new_x>=0 && new_x<rows) && (new_y>=0 && new_y<cols) && G[new_x][new_y].visited==false && G[new_x][new_y].obstacle==false){
            nbhd.push([new_x,new_y]);
        }
    }
    
    // To filter out errors rising from undefined
    if(nbhd.length>0)
        return nbhd;
    else
        return 1
}
// Function to get 8 adjacent neighbours
function get8nbhd(node) {
    var nbhd = [];
    var space = [[0,1],[1,1],[1,0],[1,-1],[0,-1],[-1,-1],[-1,0],[-1,1]];
    for(var i=0; i< space.length; i++) {
        var new_x = node[0] + space[i][0];
        var new_y = node[1] + space[i][1];
        if((new_x>=0 && new_x<rows) && (new_y>=0 && new_y<cols) && testCollision([G[new_x][new_y].x , G[new_x][new_y].y])==false){
            nbhd.push([new_x,new_y]);
        }
    }
    // To filter out errors rising from undefined
    if(nbhd.length>0)
        return nbhd;
    else
        return 1
}
// Function to get the heuristic
function heuristic(node1,node2,version) {
    var x1 = node1[0]; var y1 = node1[1];
    var x2 = node2[0]; var y2 = node2[1];
    var dx = Math.abs(x2-x1);
    var dy = Math.abs(y2-y1);
    var hue;
    switch(version) {
        case "octile":
        hue = dx+dy+(Math.sqrt(2) - 2)*Math.min(dx, dy);
        break;

        case "chebyshev":
        hue = dx+dy- Math.min(dx, dy);
        break;

        case "Manhattan":
        hue = dx+dy;
        break;

        default:
        console.warn('Using Euclidean Heuristic as default');
        hue = Math.sqrt(dx*dx + dy*dy);
    }
    return strip(hue);
}
// Function for calculating graph movement costs
function gcost(node1,node2){
    var x1 = node1[0]; var y1 = node1[1];
    var x2 = node2[0]; var y2 = node2[1];
    var dx = Math.abs(x2-x1);
    var dy = Math.abs(y2-y1);

    if(dx==1 && dy==1)
        return 1.4   // returning cost of 1.4 for diagonal movements
    else
        return 1   // returning cost of 1.0 for cardinal movements
}
// Function for calculating precision values
function strip(number) {
    if(number<0.000001 && number>-0.000001)
        return 0;
    else {
        return parseFloat(number.toPrecision(2));
    }
}
// Function to draw path on graph
function draw_path() {
    
    var curr = goal;
    
    path.push(curr);
    
    while(curr!= start) {
        curr = G[curr[0]][curr[1]].parent;
        path.push(curr);
    }

    path_length = 0;
    var st = "Path: ";
    for(var i=path.length-1;i>=0;i--)
        st += path[i]+ " ; ";
    console.log(st);
    for(var i=0; i<path.length-1;i++) {
        var p1 = [G[path[i][0]][path[i][1]].x,G[path[i][0]][path[i][1]].y];
        var p2 = [G[path[i+1][0]][path[i+1][1]].x,G[path[i+1][0]][path[i+1][1]].y];
        draw_2D_edge_configurations(p1,p2);
        path_length += Math.sqrt(Math.pow((p1[0] - p2[0]),2) + Math.pow((p1[1] - p2[1]),2));
    }
    console.log("Path Length: ",strip(path_length));
}
