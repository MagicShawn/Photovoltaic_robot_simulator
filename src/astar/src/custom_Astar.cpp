#include "my_Astar.h"


Eigen::Vector3i my_AstarPlanner::cord2index(const Eigen::Vector3d & pt){ //坐标到地图索引
    Eigen::Vector3i idx;
    idx <<  min( max( int( (pt(0) - min_cord_x) * inv_resolution), 0), max_index_x - 1),//保证索引不超过地图边界
            min( max( int( (pt(1) - min_cord_y) * inv_resolution), 0), max_index_y - 1),
            0 ;  
    //ROS_INFO(" min_x: %.3f , min_y: %.3f , max_x: %.3f , max_y: %.3f , resolution: %.3f , inv_resolution: %.3f :",min_cord_x,min_cord_y,max_cord_x,max_cord_y,resolution,inv_resolution);
    // ROS_INFO("turn %.3f , %.3f to %d , %d",pt.x(),pt.y(),idx.x(),idx.y());                
    return idx;
}


Eigen::Vector3d my_AstarPlanner::index2cord(const Eigen::Vector3i _index){
    double cord_x = double(_index.x()+0.5)*resolution + min_cord_x;
    double cord_y = double(_index.y()+0.5)*resolution + min_cord_y;
    return Eigen::Vector3d(cord_x,cord_y,0.0);
}


void my_AstarPlanner::resetNode(GridNodePtr & _nodePtr){
    _nodePtr->id = 0;
    _nodePtr->parent = NULL;
    _nodePtr->gcost = inf;
    _nodePtr->fcost = inf;

}


void my_AstarPlanner::resetMap(){
    for(int i=0;i<max_index_x;i++){
        for(int j=0;j<max_index_y;j++){
            resetNode(GridMap[i][j]);
        }
    }
}


bool my_AstarPlanner::init(const double & _resolution,const double & _min_cord_x,const double & _min_cord_y,const int & _map_width,const int & _map_height){
    if(_resolution <= 0 || _map_width <= 0 || _map_height <= 0){
        ROS_ERROR("[planner] invalid param ! init failed");
        return false;
    }
    else{
        resolution = _resolution;
        inv_resolution = 1.0/resolution;
        min_cord_x = _min_cord_x;
        min_cord_y = _min_cord_y;
        max_index_x = _map_width;
        max_index_y = _map_height;
    
        max_cord_x = min_cord_x + max_index_x * resolution;
        max_cord_y = min_cord_y + max_index_y * resolution;

        ROS_INFO("[planner] map_width: %d map_height: %d map_resolution: %.3f ; init Grid Map ...",max_index_x,max_index_y,resolution);

        initGridMap(max_index_x,max_index_x,resolution);

        ROS_INFO("[planner] Succeed init GridMap , size : %ld",sizeof(GridMap)/sizeof(GridMap[0][0]));
    
        return true;
    }
}

void my_AstarPlanner::initGridMap(const uint32_t _width,const uint32_t _height,const double _resolution){
    GridMap = new GridNodePtr* [_width];
    for(int x=0;x<_width;x++){
        GridMap[x] = new GridNodePtr [_height];
        for(int y=0;y<_height;y++){
            Eigen::Vector3i tmpIdx(x,y,0);
            Eigen::Vector3d tmpCord = index2cord(tmpIdx);
            GridMap[x][y] = new GridNode(tmpCord,tmpIdx);
        }
    }

}


void my_AstarPlanner::setObs(const Eigen::Vector3d _obs_cord){
    double _obs_x = _obs_cord.x();
    double _obs_y = _obs_cord.y();
    if(_obs_x >= max_cord_x || _obs_x < min_cord_x || _obs_y >= max_cord_y || _obs_y < min_cord_y){
        return;
    }

    Eigen::Vector3i tmpIdx = cord2index(_obs_cord);
    // ROS_INFO("[planner] obs x: %d y: %d ",tmpIdx.x(),tmpIdx.y());
    GridMap[tmpIdx.x()][tmpIdx.y()]->obsFlag = true;
}


bool my_AstarPlanner::canbeNeighbour(const int & _idx_x , const int & _idx_y){
    if( _idx_x >= 0 && _idx_x < max_index_x && _idx_y >= 0 && _idx_y < max_index_y ){
        if(GridMap[_idx_x][_idx_y]->id != -1 && !GridMap[_idx_x][_idx_y]->obsFlag){
            return true;
        }
        else{return false;}
    }
    else{return false;}
}


double my_AstarPlanner::getHeu(const GridNodePtr & _node1Ptr,const GridNodePtr & _node2Ptr){
    // ROS_INFO("[planner] get Heuristic");
    Eigen::Vector3i node1_idx = _node1Ptr->index;
    Eigen::Vector3i node2_idx = _node2Ptr->index;

    char node1_dir = _node1Ptr->dir;
    char node2_dir = _node2Ptr->dir;

    double dx = abs(node2_idx.x() - node1_idx.x());
    double dy = abs(node2_idx.y() - node1_idx.y());
    //manhatton
    double md = dx + dy ;
    //eu
    double ed = pow((dx*dx + dy*dy),0.5);
    //dia
    double dd = dx + dy + (sqrt(2.0)-2)*min(dx,dy);

    double h = ed;
    #define _use_Tie_breaker 0//tie breaker
    #if _use_Tie_breaker
        {
            double p = 1.0/(max_index_y + max_index_x) + (sqrt(2.0)-2) * min(max_index_x, max_index_y);
            double h = h*(1.0+p);
        }
    #endif
    // ROS_WARN("[planner] cur node %d , %d to tar node %d , %d , gcost: %.3f ,fcost: %.3f",
    //     node1_idx.x(),node1_idx.y(),node2_idx.x(),node2_idx.y(),_node1.gcost,_node1.gcost+h);
    return (_node1Ptr->gcost + h);
}

//refresh neighbour node
void my_AstarPlanner::getNeighbour(const GridNodePtr & _current_nodePtr ,vector<pair<GridNodePtr,double>> & _neighbourNodeSets,vector<char> & _directionSets){
    _neighbourNodeSets.clear();
    _directionSets.clear();
    // ROS_WARN("[planner] update Neighbour node");

    Eigen::Vector3i tmpIdx = _current_nodePtr->index;
    for(int i=-1;i<2;i++){//四联通搜索
        if(i != 0){
            int nbr_x = tmpIdx.x() + i;
            int nbr_y = tmpIdx.y();
            if(canbeNeighbour(nbr_x,nbr_y)){
                double edgeCost = 1.0;
                _neighbourNodeSets.push_back(make_pair(GridMap[nbr_x][nbr_y],edgeCost));
                if(i == -1){_directionSets.push_back('L');}
                else if(i == 1){_directionSets.push_back('R');}
                // ROS_INFO("[planner] neighbour node %d , %d ,dir is : %c ",nbr_x,nbr_y,_directionSets.back());
            }
        }
    }
    for(int i=-1;i<2;i++){
        if(i != 0){
            int nbr_x = tmpIdx.x();
            int nbr_y = tmpIdx.y() + i;
            if(canbeNeighbour(nbr_x,nbr_y)){
                double edgeCost = 1.0;
                _neighbourNodeSets.push_back(make_pair(GridMap[nbr_x][nbr_y],edgeCost));
                if(i == -1){_directionSets.push_back('D');}
                else if(i == 1){_directionSets.push_back('U');}
                // ROS_INFO("[planner] neighbour node %d , %d ,dir is : %c ",nbr_x,nbr_y,_directionSets.back());
            }
        }
    }
}


void my_AstarPlanner::AstarGraphSearch(const Eigen::Vector3d _st_pt,const Eigen::Vector3d _tar_pt){
    
    ros::Time st_time = ros::Time::now();

    //get index of st and tar point
    Eigen::Vector3i st_idx = cord2index(_st_pt);
    Eigen::Vector3i tar_idx = cord2index(_tar_pt);

    GridNodePtr st_nodePtr = GridMap[st_idx.x()][st_idx.y()];
    GridNodePtr tar_nodePtr = GridMap[tar_idx.x()][tar_idx.y()];
    //type multimap
    openList.clear();

    GridNodePtr current_nodePtr=NULL,neighbour_nodePtr=NULL;

    //put st node in openList
    st_nodePtr->gcost = 0;
    st_nodePtr->fcost = getHeu(st_nodePtr,tar_nodePtr);

    st_nodePtr->id = 1;
    st_nodePtr->dir = 'R';

    openList.insert( make_pair(st_nodePtr->fcost , st_nodePtr));

    // vector<GridNode> neighbourNodeSets;
    // vector<double> edgeCostSets;
    vector<pair<GridNodePtr,double>> neighbourNodeSets;
    vector<char> directionSets;

    while( !openList.empty()){
        // ROS_INFO("[planner] openlist size : %ld ",int(openList.size()) );
        // for (auto it = openList.begin(); it != openList.end(); ++it) {
        //     std::cout << "Fcost: " << it->first 
        //               << ", GridNode idx: " << it->second->index.x() << " , "<< it->second->index.y()
        //                << std::endl;
        // }123456666666666666666666
        
        current_nodePtr = openList.begin()->second;//取出当前f值最小的节点
        // ROS_INFO("[planner] current node idx : %d , %d ",current_nodePtr->index.x(),current_nodePtr->index.y());
        openList.begin()->second->id = -1 ;
        // ROS_INFO("[planner] current node id %d : openlist node id : %d , start node id : %d",current_nodePtr->id,openList.begin()->second->id,st_nodePtr->id);
        openList.erase(openList.begin());

        if(current_nodePtr->index == tar_idx){
            ros::Time end_time = ros::Time::now();
            term_nodePtr = current_nodePtr;
            ROS_WARN("[A*]{sucess}  Time in A*  is %f ms, path cost is %f m", (end_time - st_time).toSec()*1000, current_nodePtr->fcost * resolution);
            return;
        }
        //update neighbour
        getNeighbour(current_nodePtr,neighbourNodeSets,directionSets);
        //
        for(int i=0;i < (int)neighbourNodeSets.size();i++){
            neighbour_nodePtr = neighbourNodeSets[i].first;
            // ROS_INFO("[planner] neighbour node idx : %d , %d ",neighbour_nodePtr->index.x(),neighbour_nodePtr->index.y());

            if(neighbour_nodePtr->id == 0){//new node
                // ROS_INFO("[planner] neighbour node dir is : %c ",directionSets[i]);
                neighbour_nodePtr->dir = directionSets[i];

                neighbour_nodePtr->parent = current_nodePtr;
                neighbour_nodePtr->gcost = current_nodePtr->gcost + neighbourNodeSets[i].second;

                if(neighbour_nodePtr->dir != current_nodePtr->dir){
                    // ROS_INFO("[planner] redirection node!");
                    neighbour_nodePtr->gcost+=0.75; //redirection panel , should choose a appropriate value
                }

                neighbour_nodePtr->fcost = getHeu(neighbour_nodePtr,tar_nodePtr);
                neighbour_nodePtr->id = 1;
                openList.insert(make_pair(neighbour_nodePtr->fcost,neighbour_nodePtr));
                continue;
            }
            else if(neighbour_nodePtr->id == 1){//existed node , update it
                // ROS_INFO("[planner] neighbour node dir is : %c ",directionSets[i]);
                neighbour_nodePtr->dir = directionSets[i];

                double tmp_gcost = neighbour_nodePtr->gcost;
                double cur_gcost = current_nodePtr->gcost + neighbourNodeSets[i].second;

                if(neighbour_nodePtr->dir != current_nodePtr->dir){
                    // ROS_INFO("[planner] redirection node!");
                    neighbour_nodePtr->gcost+=0.75; //redirection panel , should choose a appropriate value
                }

                if(cur_gcost < tmp_gcost){
                    neighbour_nodePtr->parent = current_nodePtr;
                    neighbour_nodePtr->gcost = cur_gcost;
                    neighbour_nodePtr->fcost = getHeu(neighbour_nodePtr,tar_nodePtr);
                    openList.insert(make_pair(neighbour_nodePtr->fcost,neighbour_nodePtr));
                }
                continue;
            }
            else{
                // ROS_INFO("[planner] visited node id = -1");
                continue;
            }
        }
    }
    //if search fails
    ros::Time fail_time = ros::Time::now();
    if((fail_time - st_time).toSec() > 0.1)
        ROS_WARN("Time consume in Astar path finding is %f", (fail_time - st_time).toSec() );
}

vector<Eigen::Vector3d> my_AstarPlanner::getPath(){
    vector<Eigen::Vector3d> path;
    vector<GridNodePtr> grid_path;

    ROS_WARN("[planner] getting path");
    
    auto nodePtr = term_nodePtr;//如果使用GridNode传递，由于没有拷贝构造，易出现内存错误

    while(nodePtr->parent != NULL){
        grid_path.push_back(nodePtr);
        nodePtr = nodePtr->parent;
    }
    for (auto gridnode : grid_path){
        path.push_back(gridnode->cord);
    }
    reverse(path.begin(),path.end());

    return path;
}

vector<Eigen::Vector3d> my_AstarPlanner::pathFinding(const Eigen::Vector3d _st_pt , const Eigen::Vector3d _tar_pt){
    Eigen::Vector3i goalIndex = cord2index(_tar_pt);
    Eigen::Vector3i stIndex = cord2index(st_pt);
    ROS_WARN("[planner] start cord : %.3f , %.3f  || start index : %d , %d ",
        st_pt.x() , st_pt.y() , stIndex.x() , stIndex.y());
    ROS_WARN("[planner] goal cord : %.3f , %.3f  || goal index : %d , %d ",
        _tar_pt.x() , _tar_pt.y() , goalIndex.x() , goalIndex.y());
    ROS_WARN("[planner] start path finding");
    AstarGraphSearch(_st_pt,_tar_pt);

    auto grid_path = getPath();
    ROS_WARN("[planner] clean map");
    resetMap();
    return grid_path;
}

