// 2025.4.2 maxliang
// solar panel array map
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
    ros::init(argc,argv,"solarpanel_map");
    ros::NodeHandle nh("~");

    ros::Publisher map_pub = nh.advertise<sensor_msgs::PointCloud2>("solarpanel_array",10);

    pcl::PointCloud<pcl::PointXYZ> m_cloud;
    sensor_msgs::PointCloud2 m_pub_cloud;
    int point_nums;
    //map size 50 * 50 
    for(int i = 5;i<44;i++){
        if(i == 16 || i == 31){ i += 3;}
        for(int j = 5;j<15;j++){
            pcl::PointXYZ point(double(i+0.5),double(j+0.5),0.0);
            m_cloud.points.push_back(point);
            point_nums++;
        }
        for(int j = 35;j<45;j++){
            pcl::PointXYZ point(double(i+0.5),double(j+0.5),0.0);
            m_cloud.points.push_back(point);
            point_nums++;
        }
    }
    for(int i = 5;i<44;i++){
        for(int j = 20;j<30;j++){
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