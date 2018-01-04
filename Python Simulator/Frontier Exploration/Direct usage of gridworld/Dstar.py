from heapq import *


maxint=9999

def readmap(filename):
    fp=open(filename,"rb")
    r=0;m=[]
    for line in fp.readlines():
        c=0;row=[]
        for i in line.strip():
            row.append(int(i))
            c+=1
        m.append(row);r+=1
    return m


def Heuristic(m,g): #input as map and goal, output as matrix with heuristic values
	nrow=len(m);ncol=len(m[0])
	(r,c)=g
	kmap=[[0]*ncol for i in range(nrow)]
	hmap=[[0]*ncol for i in range(nrow)]
	kmap[r][c]=0

	for i in range(nrow):
		for j in range(ncol):
			(x,y)=(i,j)
			dr=abs(r-x);dc=abs(c-y);
			if dr<dc:
				kmap[i][j]+=1.4*dr+(dc-dr)
				if m[i][j]==1:
					hmap[i][j]=kmap[i][j]+maxint
				else:
					hmap[i][j]=kmap[i][j]
			else:
				kmap[i][j]+=1.4*dc+(dr-dc)
				if m[i][j]==1:
					hmap[i][j]=kmap[i][j]+maxint
				else:
					hmap[i][j]=kmap[i][j]
	result=[];result.append(kmap);result.append(hmap)
	return result


def neighbors(n):
	l=[(0,1),(0,-1),(1,0),(-1,0),(1,1),(1,-1),(-1,1),(-1,-1)]	# adjacency list
	(x,y)=n; alist=[]
	for i in l:
		(dx,dy)=i
		if 0<=x+dx<len(m) and 0<=y+dy<len(m[0]):
			alist.append((x+dx,y+dy))
	return alist


def changestate(a):
	(x,y)=a
	if m[x][y]==0:
		m[x][y]=1
	else:
		m[x][y]=0
	return m[x][y]


def Dijkstra(m,start,goal):
	h=[];hmap=Heuristic(readmap("maze_2.txt"),(0,6))[0]
	kmap=hmap;(srcx,srcy)=start;(desx,desy)=goal

	prev={};close_set=set();open_set=set();new_set=set()

	gscore={goal:0}
	fscore={goal:Heuristic(m,start)[0]}
	heappush(h,(kmap[desx][desy],goal))

	while h:
		curr=heappop(h)[1]

		if curr== start:
			path=[]

			while curr in prev:
				path.append(curr)
				curr=prev[curr]
			path.append(goal)

			for i in h:
				(c,p)=i
				open_set.add(p)
			return path,cost

		close_set.add(curr)
		a=neighbors(curr)
		for i in a:
			(x,y)=curr
			(nx,ny)=i
			temp=kmap[x][y]+kmap[nx][ny]
			if i in close_set and temp>=gscore.get(i,0):
				continue
			if temp<gscore.get(i,0) or i not in [j[1] for j in h]:
				prev[i]=curr
				gscore[i]=temp
				fscore[i]=temp + kmap[nx][ny]
				heappush(h,(fscore[i],i))
		(cost,pt)=h[0]
	return False

def Dstar(a,hmap,kmap,c=(3,3)):
	(path,cost)=a
	hnew=[];n=[];newpath=[];pnew=[]
	changestate(c)
	for i in range(len(path)):
		
		(x,y)=path[i]
		prev=path[i-2]
		
		if m[x][y]==0:
			hnew.append(path[i])
		
		else:
			curr=path[i-1]
			alist=neighbors(curr)
			plist=neighbors(prev)
			
			for j in alist:
				(x,y)=j
				if m[x][y]==0:
					n.append(j)
					for k in plist:
						if k==j:
							hmap[x][y]+=10000
							n.remove(k)

			n.remove(prev)

			ls=maxint
			for t in n:
				(x,y)=t
				if kmap[x][y]<ls:
					ls==kmap[x][y]
					temp=t
			(pnew,cnew)=Dijkstra(m,temp,(0,6))
			break
	return hnew+pnew


if __name__=='__main__':
	m=readmap("maze_2.txt")
	h,k=Heuristic(m,(0,6))

	for i in range(len(h)):
		for j in range(len(h[0])):
			print h[i][j],
		print
	print '\n'
	(path,cost)=Dijkstra(m,(5,1),(0,6))

	print('Old Path= {0} \nNew Path= {1} \n'.format(path,Dstar(Dijkstra(m,(5,1),(0,6)),h,k)))