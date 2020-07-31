from heapq import *

class AStar:
    def __init__(self):
        pass

    def convert(self,gridworld):
        h=gridworld.height;w=gridworld.width
        a=[[0]*w for i in range(h)]
        for obstacle in gridworld.obstacles:
            print(obstacle)
            a[obstacle[0]][obstacle[1]]=1
        return a

    def heuristic(self, a, b, n=4):
        dx=abs(b[0]-a[0])
        dy=abs(b[1]-a[1])

        # Manhattan Heuristic
        if n==1:
            return dx+dy

        # Euclidean Heuristic
        elif n==2:
            return (dx**2 + dy**2)**0.5

        # Chebychev Heuristic
        elif n==3:
            return max(dx,dy)

        # Octile Heuristic
        else: 
            return dx+dy+(2**0.5-2)*min (dx,dy)

    def aStarSearch(self,gridworld, start, goal):
        print(start,goal)
        path=[];cost2={};(x,y)=goal

       #for case when infinite map is given
        if x>gridworld.height or y>gridworld.width:
            return path,cost2

        array=self.convert(gridworld)
        close_set = set()
        came_from = {}
        gscore = {start:0}
        fscore = {start:self.heuristic(start, goal)}
        oheap = []
        heappush(oheap, (fscore[start], start))

        
        while oheap:
            current = heappop(oheap)[1]
            if current == goal:
                path = []
                cost = 0;
                arr = {}

                # make dictionary to store path and calculate cost
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                    cost+=1
                    arr[current]=cost

                l=len(arr)
                
                # return costs
                for c in arr:
                    arr[c]=l-arr[c]
                arr[goal]=l
                return (path[::-1],arr)

            close_set.add(current)

            for neighbor in gridworld.get8Neighbors(current):
                tentative_g_score = gscore[current] + self.heuristic(current, neighbor)
                if 0 <= neighbor[0] < gridworld.height:
                    if 0 <= neighbor[1] < gridworld.width:                
                        if array[neighbor[0]][neighbor[1]] == 1:
                            continue
                    
                if neighbor in close_set and tentative_g_score >= gscore.get(neighbor, 0):
                    continue
                    
                if  tentative_g_score < gscore.get(neighbor, 0) or neighbor not in [i[1]for i in oheap]:
                    came_from[neighbor] = current
                    gscore[neighbor] = tentative_g_score
                    fscore[neighbor] = tentative_g_score + self.heuristic(neighbor, goal)
                    heappush(oheap, (fscore[neighbor], neighbor))
        return False

