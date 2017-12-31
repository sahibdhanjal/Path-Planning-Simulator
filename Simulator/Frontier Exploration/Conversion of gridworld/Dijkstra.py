from heapq import *
#Dijkstra algorithm 

class Dijkstra:
    def __init__(self):
        pass

    def convert(self,gridworld):
        h=gridworld.height;w=gridworld.width
        a=[[0]*w for i in range(h)]
        for obstacle in gridworld.obstacles:
            a[obstacle[0]][obstacle[1]]=1
        return a

    def aStarSearch(self,gridworld, start, goal):
        path=[];cost2={};(x,y)=goal

        #for case when infinite map is given
        if x>gridworld.height or y>gridworld.width:
            return path,cost2
            
        array=self.convert(gridworld)
        close_set = set()
        came_from = {}
        gscore = {start:0}
        oheap = []
        heappush(oheap, (gscore[start], start))
        
        while oheap:
            current = heappop(oheap)[1]
            if current == goal:
                path = [] ; cost = 0 ; arr = {}
                while current in came_from:
                    path.append(current) ; current = came_from[current] ; cost+=1 ; arr[current]=cost

                l=len(arr)
                for c in arr:
                    arr[c]=l-arr[c]
                arr[goal]=l
                return (path[::-1],arr)

            close_set.add(current)

            for neighbor in gridworld.get8Neighbors(current):
                if neighbor[0]==current[0] or neighbor[1]==current[1]:
                    tentative_g_score = gscore[current]+1
                else:
                    tentative_g_score = gscore[current]+1.4
                    
                if neighbor in close_set and tentative_g_score >= gscore.get(neighbor, 0):
                    continue
                    
                if  tentative_g_score < gscore.get(neighbor, 0) or neighbor not in [i[1]for i in oheap]:
                    came_from[neighbor] = current
                    gscore[neighbor] = tentative_g_score 
                    heappush(oheap, (gscore[neighbor], neighbor))
        return False

