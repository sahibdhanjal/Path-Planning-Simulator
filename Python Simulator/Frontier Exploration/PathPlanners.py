from Datatypes import *
import itertools, heapq

class FastPriorityQueue:
    def __init__(self):
        self.pq = []                         # list of entries arranged in a heap
        self.counter = itertools.count()     # unique sequence count

    def put(self, task, priority=0):
        count = next(self.counter)
        entry = [priority, count, task]
        heapq.heappush(self.pq, entry)

    def get(self):
        while self.pq:
            priority, count, task = heapq.heappop(self.pq)
            return task
        raise KeyError('pop from an empty priority queue')

    def isEmpty(self):
        return len(self.pq) == 0


# --------------------------Breadth First Search-------------------- #
class BFS:
    def __init__(self):
        pass

    def BFSsearch(self,gridworld,start,goal):
        frontier=Queue()
        came_from={} 
        frontier.put(start) ; came_from[start] = None
        path=[] ; cost={}
        cost[start] = 0

        while not frontier.isEmpty():
            current=frontier.get()

            if current==goal:
                break
            for next in gridworld.get8Neighbors(current):
                if next not in came_from:
                    frontier.put(next)
                    came_from[next]=current
                    cost[next] = cost[current]+5
        
        current=goal
        while current!=start:
            path.append(current)
            current=came_from[current]
        path=[start]+path[::-1]
        return path,cost


#---------------------------Dijkstra Search------------------------- #

class Dijkstra:
    def __init__(self):
        pass

    def DijkstraSearch(self,gridworld,start,goal):
        frontier=PriorityQueue() ; frontier.put(start,0)
        came_from={} ; cost={} ; path=[]
        came_from[start]=None ; cost[start]=0

        while not frontier.isEmpty():
            current = frontier.get()

            if current==goal:
                break

            for next in gridworld.get8Neighbors(current):
                if next[0]==current[0] or next[1] == current[1]:
                    newcost=cost[current]+5
                else:
                    newcost=cost[current]+7

                if next not in cost or newcost<cost[next]:
                    cost[next]=newcost
                    priority = newcost
                    frontier.put(next, priority)
                    came_from[next]=current

        curr=goal

        while curr!=start:
            path.append(curr)
            curr=came_from[curr]
        path=[start]+path[::-1]
        return path,cost


# ------------ A* and Dynamic Heuristic A* Algorithm----------------#

class AStar:
    def __init__(self):
        pass

    def heuristics(self, a, b, flag=1):
        dx=abs(b[0]-a[0])
        dy=abs(b[1]-a[1])

        # Manhattan Heuristic
        if flag==1:
            return dx+dy

        # Euclidean Heuristic
        elif flag==2:
            return (dx**2 + dy**2)**0.5

        # Chebychev Heuristic
        elif flag==3:
            return max(dx,dy)

        # Octile Heuristic
        else: 
            return dx+dy+(2**0.5-2)*min (dx,dy)


    def AStarSearch(self, gridworld,start,goal,dynamic=True):
        frontier=PriorityQueue() ; frontier.put(start,0)
        came_from={} ; cost={} ; path=[]
        came_from[start]=None ; cost[start]=0
        flag=0

        while not frontier.isEmpty():
            current = frontier.get()

            if current==goal:
                flag=1
                break

            for next in gridworld.get8Neighbors(current):
                if next[0]==current[0] or next[1] == current[1]:
                    newcost=cost[current]+5
                else:
                    newcost=cost[current]+7

                if next not in cost or newcost<cost[next]:
                    cost[next]=newcost

                    # dynamic weighting added to a-star algorithm for achieving results faster
                    if not dynamic:
                        priority = newcost + self.heuristics(next,goal)
                    else:
                        if self.heuristics(next,goal)<5:
                            w=1
                        else:
                            w=5
                        priority = newcost + w * self.heuristics(next,goal)

                    frontier.put(next, priority)
                    came_from[next]=current

        curr=goal

        while curr!=start:
            path.append(curr)
            curr=came_from[curr]
        path=[start]+path[::-1]
        return path,cost


# -----------------------Greedy Best First Search ---------------------#

class GreedyBestFirstSearch:
    def __init__(self):
        pass

    def heuristics(self, a, b, flag=3):
        dx=abs(b[0]-a[0])
        dy=abs(b[1]-a[1])

        # Manhattan Heuristic
        if flag==1:
            return dx+dy

        # Euclidean Heuristic
        elif flag==2:
            return (dx**2 + dy**2)**0.5

        # Chebychev Heuristic
        elif flag==3:
            return max(dx,dy)

        # Octile Heuristic
        else: 
            return dx+dy+(2**0.5-2)*min (dx,dy)

    def GreedySearch(self,gridworld,start,goal):
        frontier = PriorityQueue()
        frontier.put(start,0)
        came_from = {}
        came_from[start] = None
        path=[] ; cost=0
        while not frontier.isEmpty():
           current = frontier.get()

           if current == goal:
              break
           
           for next in gridworld.get8Neighbors(current):
              if next not in came_from:
                 priority = self.heuristics(goal, next)
                 frontier.put(next,priority)
                 came_from[next] = current
                 if next[0]==current[0] or next[1] == current[1]:
                    cost+=5
                 else:
                    cost+=7
        
        curr=goal

        while curr!=start:
            path.append(curr)
            curr=came_from[curr]
        path=[start]+path[::-1]

        return path,cost

# ------------------------Jump Point Search ---------------------------------#

# global variables required for the operation of the JPS algorithm
grid = [] ; expanded = [] ; visited = [] ; h = 0 ; w = 0 

class JPS:
    def __init__(self):
        pass

    def construct(self,sources,start,goal):
        jump_points=[]
        curr_x,curr_y=goal

        while curr_x!=start[0] or curr_y!=start[1]:
            jump_points.append((curr_x,curr_y))
            curr_x,curr_y=sources[curr_x][curr_y]
            jump_points.reverse()
        return [start]+jump_points


    def full_path(self,a):
        '''
        Take individual jump points as a list of tuples and return full path

        RETURN:
        Full pathof traversal or [] if no path is found
        '''
        a.sort()
        if a==[]:
            return []

        path=[] ; l=len(a) ; cost={} ; c=0

        for i in range(l-1):
            path.append(a[i])

            cost[a[i]]=c

            curr_x=a[i][0] ; curr_y=a[i][1] ; next_x=a[i+1][0] ; next_y=a[i+1][1]
            dx=next_x-curr_x ; dy=next_y-curr_y

            if dy==0:
                if dx > 0:
                    for j in range(1,dx):
                        c+=5
                        path.append((curr_x+j,curr_y))
                        cost[(curr_x+j,curr_y)]=c
                if dx < 0:
                    for j in range(dx+1,0):
                        c+=5
                        path.append((curr_x+j,curr_y))
                        cost[(curr_x+j,curr_y)]=c

            if dx==0:
                if dy >0:
                    for j in range(1,dy):
                        c+=5
                        path.append((curr_x,curr_y+j))
                        cost[(curr_x,curr_y+j)]=c
                if dy < 0:
                    for j in range(dy+1,0):
                        c+=5
                        path.append((curr_x,curr_y+j))
                        cost[(curr_x,curr_y+j)]=c
            
            if abs(dx) == abs(dy) and abs(dx)>1:
                if dx>0 and dy>0:
                    for j in range(1,dx):
                        c+=7
                        path.append((curr_x+j,curr_y+j))
                        cost[(curr_x+j,curr_y+j)]=c
                elif dx<0 and dy<0:
                    for j in range(dx+1,0):
                        c+=7
                        path.append((curr_x+j,curr_y+j))
                        cost[(curr_x+j,curr_y+j)]=c
                elif dx>0 and dy<0:
                    for j in range(1,dx):
                        c+=7
                        path.append((curr_x+j,curr_y-j))
                        cost[(curr_x+j,curr_y-j)]=c
                else:
                    for j in range(1,dx):
                        c+=7
                        path.append((curr_x-j,curr_y+j))
                        cost[(curr_x-j,curr_y+j)]=c

        path.append(a[l-1])
        return(path,cost)


    def jps(self,gridworld,start,goal):
        '''
        Run a Jump point search on a field with obstacles

        gridworld - 2d array representing obstacles as 1 and traversable paths as -1
        grid - 2d array representing the cost to get to that node
        start - starting point for the search
        goal - finishing point for the search

        Return:
        List of Jump Points as Tuples OR [] if no path is found
        '''
        class FoundPath(Exception):
            pass

        global grid , expanded , visited , h , w

        h=gridworld.height+1;w=gridworld.width+1

        grid = [[0]*w for i in range(h)]

        for i in range(h):
            grid[i][0]=1 ; grid[i][-1]=1
        for j in range(w):
            grid[0][j]=1 ; grid[-1][j]=1


        expanded = [[False]*w for i in range(h)]
        visited = [[False]*w for i in range(h)]
        sources = [[(None,None)]*w for i in range(h)]

        for obstacle in gridworld.obstacles:
            grid[obstacle[0]][obstacle[1]]=1

        start_x , start_y = start
        goal_x , goal_y = goal

        if start_x==0:
            start_x+=1
        
        if start_y==0:
            start_y+=1

        if grid[start_x][start_y]==1:
            raise ValueError("No path exists: start point is an obstacle")
        if  grid[goal_x][goal_y]==1:
            raise ValueError("No path exists: goal point is an obstacle")

        def add_jump_point(node):
            if node is not None:
                p.put(node , grid[node[0]][node[1]] + max(abs(node[0]-goal_x),abs(node[1]-goal_y)))

        def diagonal(start,dirn):
            curr_x,curr_y=start[0],start[1]
            dir_x,dir_y=dirn

            curCost=grid[start[0]][start[1]]

            while (True):
                curr_x+=dir_x ; curr_y+=dir_y ; curCost +=1

                if grid[curr_x][curr_y] == 0:
                    grid[curr_x][curr_y]=curCost
                    sources[curr_x][curr_y]=start
                    visited[curr_x][curr_y]=True

                # Destination found
                elif curr_x==goal_x and curr_y==goal_y:
                    grid[curr_x][curr_y]=curCost
                    sources[curr_x][curr_y]=start
                    visited[curr_x][curr_y]=True
                    raise FoundPath()
                
                # collided with obstacle
                else:
                    return None

                # If jump point is found
                if grid[curr_x+dir_x][curr_y] == 1 and grid[curr_x+dir_x][curr_y+dir_y]!=1:
                    return (curr_x,curr_y)
                # Extend horizontal search
                else:
                    add_jump_point(cardinal((curr_x,curr_y),(dir_x,0)))

                # If jump point is found
                if grid[curr_x][curr_y+dir_y]==1 and grid[curr_x+dir_x][curr_y+dir_y]!=1:
                    return (curr_x,curr_y)
                #Entend vertical search
                else:
                    add_jump_point(cardinal((curr_x,curr_y),(0,dir_y)))

        def cardinal(start,dirn):
            curr_x,curr_y=start
            dir_x,dir_y=dirn

            curCost=grid[start[0]][start[1]]

            while (True):
                curr_x+=dir_x ; curr_y+=dir_y ; curCost+=1

                if grid[curr_x][curr_y] == 0:
                    grid[curr_x][curr_y]=curCost
                    sources[curr_x][curr_y]=start
                    visited[curr_x][curr_y]=True

                # Destination found
                elif curr_x==goal_x and curr_y==goal_y:
                    grid[curr_x][curr_y]=curCost
                    sources[curr_x][curr_y]=start
                    visited[curr_x][curr_y]=True
                    raise FoundPath()
                
                # collided with obstacle or previously explored path
                else:
                    return None


                if dir_x==0:
                    if grid[curr_x+1][curr_y]==1 and grid[curr_x+1][curr_y+dir_y]!=1:
                        return (curr_x,curr_y)
                    if grid[curr_x-1][curr_y]==1 and grid[curr_x-1][curr_y+dir_y]!=1:
                        return (curr_x,curr_y)

                elif dir_y==0:
                    if grid[curr_x][curr_y+1]==1 and grid[curr_x+dir_x][curr_y+1]!=1:
                        return (curr_x,curr_y)
                    if grid[curr_x][curr_y-1]==1 and grid[curr_x+dir_x][curr_y-1]!=1:
                        return (curr_x,curr_y)

        grid[start[0]][start[1]]=0 ; grid[goal[0]][goal[1]]=-2

        p=FastPriorityQueue()
        add_jump_point(start)

        # Main loop: iterate through queue
        while not p.isEmpty():
            pX,pY=p.get()
            expanded[pX][pY]=True

            try:
                add_jump_point(cardinal((pX,pY),(1,0)))
                add_jump_point(cardinal((pX,pY),(-1,0)))
                add_jump_point(cardinal((pX,pY),(0,1)))
                add_jump_point(cardinal((pX,pY),(0,-1)))

                add_jump_point(diagonal((pX,pY),(1,1)))
                add_jump_point(diagonal((pX,pY),(1,-1)))
                add_jump_point(diagonal((pX,pY),(-1,1)))
                add_jump_point(diagonal((pX,pY),(-1,-1)))

            except FoundPath:
                return self.construct(sources,start,goal)

        raise ValueError('No path found')