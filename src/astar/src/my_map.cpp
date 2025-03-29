// 2025.3.23 maxliang
// customize 
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

int main(int argc,char * argv[]){
    ros::init(argc,argv,"my_map");
    ros::NodeHandle nh("~");

    ros::Publisher map_pub = nh.advertise<sensor_msgs::PointCloud2>("custom_map",10);

    pcl::PointCloud<pcl::PointXYZ> m_cloud;
    sensor_msgs::PointCloud2 m_pub_cloud;
    int point_nums;

    for(int i = 4;i<7;i++){
        for(int j = 3;j<8;j++){
            pcl::PointXYZ point(double(i+0.5),double(j+0.5),0.0);
            m_cloud.points.push_back(point);
            point_nums++;
        }
    }
    m_cloud.width = m_cloud.points.size();
    m_cloud.height = 1;
    m_cloud.is_dense = true;
    pcl::toROSMsg(m_cloud,m_pub_cloud);
    m_pub_cloud.header.frame_id = "world";

    ros::Rate spinrate(1.0);
    while(ros::ok()){
        // ROS_WARN_THROTTLE(1.0,"[map_gen node] publish map!"); 
        map_pub.publish(m_pub_cloud);
        ros::spinOnce();
        spinrate.sleep();
    }
    return 0;
}