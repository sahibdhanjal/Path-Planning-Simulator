#include "pathplanners.h"

PLUGINLIB_EXPORT_CLASS(PathPlanners_all::PathPlannersROS, nav_core::BaseGlobalPlanner);

int mapSize;
bool* OGM;
static const float INFINIT_COST = INT_MAX;
float infinity = std::numeric_limits< float >::infinity();

int clock_gettime(clockid_t clk_id, struct timespect *tp);

timespec diff(timespec start, timespec end){
	
	timespec temp;
	
	if ((end.tv_nsec-start.tv_nsec)<0){
		temp.tv_sec = end.tv_sec-start.tv_sec-1;
		temp.tv_nsec = 1000000000+end.tv_nsec-start.tv_nsec;
	}
	
	else{
		temp.tv_sec = end.tv_sec-start.tv_sec;
		temp.tv_nsec = end.tv_nsec-start.tv_nsec;
	}
	return temp;
}


inline vector <int> getNeighbour (int CellID);


namespace PathPlanners_all
{
PathPlannersROS::PathPlannersROS(){}
PathPlannersROS::PathPlannersROS(std::string name, costmap_2d::Costmap2DROS* costmap_ros){initialize(name, costmap_ros);}

void PathPlannersROS::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros){
	if (!initialized_){
		costmap_ros_ = costmap_ros;
		costmap_ = costmap_ros_->getCostmap();

		originX = costmap_->getOriginX();
		originY = costmap_->getOriginY();

		width = costmap_->getSizeInCellsX();
		height = costmap_->getSizeInCellsY();
		resolution = costmap_->getResolution();
		mapSize = width*height;

		OGM = new bool [mapSize]; 
		for (unsigned int iy = 0; iy < height; iy++){
			for (unsigned int ix = 0; ix < width; ix++){
				unsigned int cost = static_cast<int>(costmap_->getCost(ix,iy));
				
				if (cost <= 255){
					OGM[iy*width+ix]=true;
					// cout <<"Traversable"<< ix<<","<<iy<<"   cost:"<<cost<<endl;
				}

				else{
					OGM[iy*width+ix]=false;
					// cout <<"Obstacle"<< ix<<","<<iy<<"   cost:"<<cost<<endl;
				}
			}
		}
		ROS_INFO("Jump Point Search initialized successfully");
		initialized_ = true;
	}
	else
    	ROS_WARN("Planner already initialized");
}



bool PathPlannersROS::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,std::vector<geometry_msgs::PoseStamped>& plan){
	if (!initialized_){
		ROS_ERROR("The planner has not been initialized, please call initialize() to use the planner");
		return false;
	}

	ROS_DEBUG("Got a start: %.2f, %.2f, and a goal: %.2f, %.2f", start.pose.position.x, start.pose.position.y,goal.pose.position.x, goal.pose.position.y);
	plan.clear();

	if (goal.header.frame_id != costmap_ros_->getGlobalFrameID()){
		ROS_ERROR("This planner as configured will only accept goals in the %s frame, but a goal was sent in the %s frame.",
		costmap_ros_->getGlobalFrameID().c_str(), goal.header.frame_id.c_str());
		return false;
	}

	tf::Stamped < tf::Pose > goal_tf;
	tf::Stamped < tf::Pose > start_tf;

	poseStampedMsgToTF(goal, goal_tf);
	poseStampedMsgToTF(start, start_tf);

	float startX = start.pose.position.x;
	float startY = start.pose.position.y;

	float goalX = goal.pose.position.x;
	float goalY = goal.pose.position.y;

	getCoordinate(startX, startY);
	getCoordinate(goalX, goalY);

	int startCell;
	int goalCell;

	if (validate(startX, startY) && validate(goalX, goalY)){
		startCell = convertToCellIndex(startX, startY);
		goalCell = convertToCellIndex(goalX, goalY);
	}

	else{
		ROS_WARN("the start or goal is out of the map");
		return false;
	}

	if (isValid(startCell, goalCell)){
		vector<int> bestPath;
		bestPath.clear();
		bestPath = PathFinder(startCell, goalCell);
		if(bestPath.size()>0){
			for (int i = 0; i < bestPath.size(); i++){
				float x = 0.0;
				float y = 0.0;
				int index = bestPath[i];
				convertToCoordinate(index, x, y);
				geometry_msgs::PoseStamped pose = goal;

				pose.pose.position.x = x;
				pose.pose.position.y = y;
				pose.pose.position.z = 0.0;

				pose.pose.orientation.x = 0.0;
				pose.pose.orientation.y = 0.0;
				pose.pose.orientation.z = 0.0;
				pose.pose.orientation.w = 1.0;

				plan.push_back(pose);
			}

			float path_length = 0.0;
			std::vector<geometry_msgs::PoseStamped>::iterator it = plan.begin();
			geometry_msgs::PoseStamped last_pose;
			last_pose = *it;
			it++;

			for (; it!=plan.end();++it){
				path_length += hypot((*it).pose.position.x - last_pose.pose.position.x, (*it).pose.position.y - last_pose.pose.position.y );
				last_pose = *it;
			}

			cout <<"The global path length: "<< path_length<< " meters"<<endl;
			return true;
		}
		else{
			ROS_WARN("The planner failed to find a path, choose other goal position");
			return false;
		}
	}
	
	else{
		ROS_WARN("Not valid start or goal");
		return false;
	}
}

void PathPlannersROS::getCoordinate(float& x, float& y){
	x = x - originX;
	y = y - originY;
}

int PathPlannersROS::convertToCellIndex(float x, float y){
	int cellIndex;
	float newX = x / resolution;
	float newY = y / resolution;
	cellIndex = getIndex(newY, newX);
	return cellIndex;
}

void PathPlannersROS::convertToCoordinate(int index, float& x, float& y){
	x = getCol(index) * resolution;
	y = getRow(index) * resolution;
	x = x + originX;
	y = y + originY;
}

bool PathPlannersROS::validate(float x, float y){
	bool valid = true;
	if (x > (width * resolution) || y > (height * resolution))
		valid = false;
	return valid;
}

void PathPlannersROS::mapToWorld(double mx, double my, double& wx, double& wy){
	costmap_2d::Costmap2D* costmap = costmap_ros_->getCostmap();
	wx = costmap->getOriginX() + mx * resolution;
	wy = costmap->getOriginY() + my * resolution;
}

vector<int> PathPlannersROS::PathFinder(int startCell, int goalCell){
	vector<int> bestPath;
	float g_score [mapSize];
	for (uint i=0; i<mapSize; i++)
		g_score[i]=infinity;

	timespec time1, time2;

	clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &time1);
	
	bestPath=AStar(startCell, goalCell,  g_score);
	// bestPath=Dijkstra(startCell, goalCell,  g_score);
	// bestPath=BFS(startCell, goalCell,  g_score);
	// bestPath=JPS(startCell,goalCell,g_score);

	clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &time2);

	cout<<" Time taken to generate path= " << (diff(time1,time2).tv_sec)*1e3 + (diff(time1,time2).tv_nsec)*1e-6 << " microseconds" << endl;

	return bestPath;
}

vector<int> PathPlannersROS::AStar(int startCell, int goalCell, float g_score[]){
	vector<int> bestPath;
	vector<int> emptyPath;
	cells CP;

	multiset<cells> OPL;
	int currentCell;

	g_score[startCell]=0;
	CP.currentCell=startCell;
	CP.fCost=g_score[startCell]+heuristic(startCell,goalCell,1);
	OPL.insert(CP);
	currentCell=startCell;

	while (!OPL.empty()&& g_score[goalCell]==infinity){
		currentCell = OPL.begin()->currentCell;
		OPL.erase(OPL.begin());
		vector <int> neighborCells; 
		neighborCells=getNeighbour(currentCell);
		for(uint i=0; i<neighborCells.size(); i++){
			if(g_score[neighborCells[i]]==infinity){
				g_score[neighborCells[i]]=g_score[currentCell]+getMoveCost(currentCell,neighborCells[i]);
				add_open(OPL, neighborCells[i], goalCell, g_score, 1); 
			}
		}
	}

	if(g_score[goalCell]!=infinity){
		bestPath=constructPath(startCell, goalCell, g_score);
		return bestPath; 
	}
	
	else{
		cout << "Path not found!" << endl;
		return emptyPath;
	}
}

vector<int> PathPlannersROS::Dijkstra(int startCell, int goalCell, float g_score[]){
	vector<int> bestPath;
	vector<int> emptyPath;
	cells CP;

	multiset<cells> OPL;
	int currentCell;

	g_score[startCell]=0;
	CP.currentCell=startCell;
	CP.fCost=g_score[startCell];
	OPL.insert(CP);
	currentCell=startCell;

	while (!OPL.empty()&& g_score[goalCell]==infinity){
		currentCell = OPL.begin()->currentCell;
		OPL.erase(OPL.begin());
		vector <int> neighborCells; 
		neighborCells=getNeighbour(currentCell);
		for(uint i=0; i<neighborCells.size(); i++){
			if(g_score[neighborCells[i]]==infinity){
				g_score[neighborCells[i]]=g_score[currentCell]+getMoveCost(currentCell,neighborCells[i]);
				add_open(OPL, neighborCells[i], goalCell, g_score, 0); 
			}
		}
	}

	if(g_score[goalCell]!=infinity){
		bestPath=constructPath(startCell, goalCell, g_score);
		return bestPath; 
	}
	
	else{
		cout << "Path not found!" << endl;
		return emptyPath;
	}
}

vector<int> PathPlannersROS::BFS(int startCell, int goalCell, float g_score[]){
	vector<int> bestPath;
	vector<int> emptyPath;
	cells CP;

	multiset<cells> OPL;
	int currentCell;

	g_score[startCell]=0;
	CP.currentCell=startCell;
	CP.fCost=g_score[startCell];
	OPL.insert(CP);
	currentCell=startCell;

	while (!OPL.empty()&& g_score[goalCell]==infinity){
		currentCell = OPL.begin()->currentCell;
		OPL.erase(OPL.begin());
		vector <int> neighborCells; 
		neighborCells=getNeighbour(currentCell);
		for(uint i=0; i<neighborCells.size(); i++){
			if(g_score[neighborCells[i]]==infinity){
				g_score[neighborCells[i]]=g_score[currentCell]+1;
				add_open(OPL, neighborCells[i], goalCell, g_score, 0); 
			}
		}
	}

	if(g_score[goalCell]!=infinity){
		bestPath=constructPath(startCell, goalCell, g_score);
		return bestPath; 
	}
	
	else{
		cout << "Path not found!" << endl;
		return emptyPath;
	}
}



vector<int> PathPlannersROS::constructPath(int startCell, int goalCell,float g_score[])
{
	vector<int> bestPath;
	vector<int> path;

	path.insert(path.begin()+bestPath.size(), goalCell);
	int currentCell=goalCell;

	while(currentCell!=startCell){ 
		vector <int> neighborCells;
		neighborCells=getNeighbour(currentCell);

		vector <float> gScoresNeighbors;
		for(uint i=0; i<neighborCells.size(); i++)
			gScoresNeighbors.push_back(g_score[neighborCells[i]]);
		
		int posMinGScore=distance(gScoresNeighbors.begin(), min_element(gScoresNeighbors.begin(), gScoresNeighbors.end()));
		currentCell=neighborCells[posMinGScore];

		path.insert(path.begin()+path.size(), currentCell);
	}

	for(uint i=0; i<path.size(); i++)
		bestPath.insert(bestPath.begin()+bestPath.size(), path[path.size()-(i+1)]);

	return bestPath;
}

void PathPlannersROS::add_open(multiset<cells> & OPL, int neighborCell, int goalCell, float g_score[] ,int n){
	cells CP;
	CP.currentCell=neighborCell;
	if (n==1)
		CP.fCost=g_score[neighborCell]+heuristic(neighborCell,goalCell,1);
	else
		CP.fCost=g_score[neighborCell];
	OPL.insert(CP);
}

vector <int> PathPlannersROS::getNeighbour (int CellID){
	int rowID=getRow(CellID);
	int colID=getCol(CellID);
	int neighborIndex;
	vector <int>  freeNeighborCells;

	for (int i=-1;i<=1;i++)
		for (int j=-1; j<=1;j++){
			if ((rowID+i>=0)&&(rowID+i<height)&&(colID+j>=0)&&(colID+j<width)&& (!(i==0 && j==0))){
				neighborIndex = getIndex(rowID+i,colID+j);
				if(isFree(neighborIndex) )
					freeNeighborCells.push_back(neighborIndex);
			}
		}

	return  freeNeighborCells;
}

bool PathPlannersROS::isValid(int startCell,int goalCell){ 
	bool isvalid=true;
	
	bool isFreeStartCell=isFree(startCell);
	bool isFreeGoalCell=isFree(goalCell);
	
	if (startCell==goalCell){
		cout << "The Start and the Goal cells are the same..." << endl; 
		isvalid = false;
	}
	
	else{
		
		if(!isFreeStartCell && !isFreeGoalCell){
			cout << "The start and the goal cells are obstacle positions..." << endl;
			isvalid = false;
		}
		
		else{
			if(!isFreeStartCell){
				cout << "The start is an obstacle..." << endl;
				isvalid = false;
			}
			else{
				if(!isFreeGoalCell){
					cout << "The goal cell is an obstacle..." << endl;
					isvalid = false;
				}
				else{
					if (getNeighbour(goalCell).size()==0){
						cout << "The goal cell is encountred by obstacles... "<< endl;
						isvalid = false;
					}
					else{
						if(getNeighbour(startCell).size()==0){
							cout << "The start cell is encountred by obstacles... "<< endl;
							isvalid = false;
						}
					}
				}
			}
		}
	}

return isvalid;
}

float  PathPlannersROS::getMoveCost(int CellID1, int CellID2){
	int i1=0,i2=0,j1=0,j2=0;

	i1=getRow(CellID1);
	j1=getCol(CellID1);
	i2=getRow(CellID2);
	j2=getCol(CellID2);

	float moveCost=INFINIT_COST;
	if(i1!=i2 && j1!=j2)
		moveCost=1.4;
	else
		moveCost=1;
	return moveCost;
} 

bool  PathPlannersROS::isFree(int CellID){
	return OGM[CellID];
} 
};

bool operator<(cells const &c1, cells const &c2){return c1.fCost < c2.fCost;}
