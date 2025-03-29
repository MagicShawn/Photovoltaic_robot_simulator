// 2025.3.08 maxliang
// API that load 2d map , need a node to load map first (usually map_server)
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <iostream>
#include <Eigen/Dense>
#include <cmath>
#include <string>
#include <vector>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/search/kdtree.h>
#include <pcl/search/impl/kdtree.hpp>

using namespace std;

sensor_msgs::PointCloud2 g_obs_points;

void fillDia(const nav_msgs::OccupancyGrid & _map,const int _x,const int _y,vector<Eigen::Vector3d> & _obs_cloud){
    uint32_t m_width = _map.info.width;
    int tmp_y = _y-1;
    if(tmp_y <= 0){return;}
    if(_map.data[tmp_y*m_width+(_x-1)]!=0 || _map.data[tmp_y*m_width+(_x+1)]!=0){
        double cell_center_x = _map.info.origin.position.x + (_x + 0.5) * _map.info.resolution;
        double cell_center_y = _map.info.origin.position.y + (tmp_y + 0.5) * _map.info.resolution;
        double cell_center_z = 0.0;
        _obs_cloud.push_back(Eigen::Vector3d(cell_center_x,cell_center_y,cell_center_z));
    }
}

//地图转换为图搜索
//接受SLAM得到的建图数据 并转换为二维的点云数据
void rcvMapCallBack(const nav_msgs::OccupancyGrid& _map){

    Eigen::Vector3d origin;
    Eigen::Vector3i ori_index;
    
    double resolution = _map.info.resolution;
    double inv_resolution = 1/_map.info.resolution;
    uint32_t m_width,m_height;
    m_width = _map.info.width;
    m_height = _map.info.height;
    origin << _map.info.origin.position.x, _map.info.origin.position.y,_map.info.origin.position.z;
    ori_index  << static_cast<int32_t>(origin.x() * inv_resolution) , static_cast<int32_t>(origin.y() * inv_resolution),static_cast<int32_t>(origin.z() * inv_resolution);
    
    // Debug Info
    ROS_WARN_THROTTLE(1.0,"[map_gen node] Map Width : %ld , Map Height : %ld , Resolution : %.3f ,inv_Resolution : %.3f , Size : %ld",
        _map.info.width,_map.info.height,_map.info.resolution,inv_resolution,_map.data.size());
    ROS_WARN_THROTTLE(1.0,"[map_gen node] Origin of Map %.3f , %.3f , %.3f; Origin Index of Map %d , %d , %d",
        origin.x(),origin.y(),origin.z(),ori_index.x(),ori_index.y(),ori_index.z()); 
    //load obstacle points
    vector<Eigen::Vector3d> m_obs_points;
    pcl::PointCloud<pcl::PointXYZ> m_obs_cloud;
    for(int y=0;y<m_height;++y){
        for(int x=0;x<m_width;++x){
            if(_map.data[y*m_width + x] != -1 && _map.data[y*m_width + x] != 0){
                double cell_center_x = origin.x() + (x+0.5)*resolution; 
                double cell_center_y = origin.y() + (y+0.5)*resolution; 
                double cell_center_z = 0.0;
                fillDia(_map,x,y,m_obs_points);
                m_obs_points.push_back(Eigen::Vector3d(cell_center_x,cell_center_y,cell_center_z));
            }
        }
    }
    ROS_WARN_THROTTLE(1.0,"[map_gen node] there are %d obs points in the map",m_obs_points.size()); 
    m_obs_cloud.points.resize(m_obs_points.size());
    for(int i=0;i<m_obs_points.size();i++){
        m_obs_cloud.points[i].x = m_obs_points[i].x();
        m_obs_cloud.points[i].y = m_obs_points[i].y();
        m_obs_cloud.points[i].z = m_obs_points[i].z();
    }
    m_obs_cloud.width = m_obs_cloud.points.size();
    m_obs_cloud.height = 1;
    m_obs_cloud.is_dense = true;
    pcl::toROSMsg(m_obs_cloud,g_obs_points);
    g_obs_points.header.frame_id = "world";

}


int main(int argc,char* argv[]){

    ros::init(argc,argv,"map_generator");
    ros::NodeHandle nh("~");

    ros::Publisher  graph_pub = nh.advertise<sensor_msgs::PointCloud2>("global_map",10) ;
    ros::Subscriber map_sub = nh.subscribe("/map",1,rcvMapCallBack);
    
    ros::Rate loop_rate(1.0);
    while(ros::ok()){
        // ROS_WARN_THROTTLE(1.0,"[map_gen node] publish map!"); 
        graph_pub.publish(g_obs_points);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}