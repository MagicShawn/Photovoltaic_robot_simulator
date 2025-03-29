#ifndef _MY_ASTAR_H_
#define _MY_ASTAR_H_

#include "my_GridNode.h"

#include <iostream>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>

#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <math.h>
#include <vector>
#include <set>

using namespace std;

class my_AstarPlanner{
private:
    double resolution,inv_resolution,min_cord_x,min_cord_y,max_cord_x,max_cord_y;
    int max_index_x,max_index_y;

    GridNodePtr ** GridMap;

    Eigen::Vector3d st_pt;
    GridNodePtr term_nodePtr;
    std::multimap<double, GridNodePtr> openList;

protected:
    bool canbeNeighbour(const int & _idx_x , const int & _idx_y);
    double getHeu(const GridNodePtr & _node1Ptr,const GridNodePtr & _node2Ptr);
    void getNeighbour(const GridNodePtr & _current_nodePtr ,vector<pair<GridNodePtr,double>> & _neighbourNodeSets);

    Eigen::Vector3i cord2index(const Eigen::Vector3d & pt);
    Eigen::Vector3d index2cord(const Eigen::Vector3i _index);
    void resetNode(GridNodePtr & _nodePtr);
    void resetMap();
    void initGridMap(const uint32_t _width,const uint32_t _height,const double _resolution);    

    void AstarGraphSearch(const Eigen::Vector3d _st_pt,const Eigen::Vector3d _tar_pt);

    vector<Eigen::Vector3d> getPath();

public:
    my_AstarPlanner(){};
    ~my_AstarPlanner(){};

    my_AstarPlanner(const double & _resolution,const double & _min_cord_x,const double & _min_cord_y,const int & _map_width,const int & _map_height){
        init(_resolution,_min_cord_x,_min_cord_y,_map_width,_map_height);
    };

    vector<Eigen::Vector3d> pathFinding(const Eigen::Vector3d _st_pt , const Eigen::Vector3d _tar_pt);
    void setObs(const Eigen::Vector3d _obs_cord);
    
    bool init(const double & _resolution,const double & _min_cord_x,const double & _min_cord_y,const int & _map_width,const int & _map_height);
};

#endif
