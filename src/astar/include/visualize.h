#ifndef VISUALIZE_H
#define VISUALIZE_H

#include <vector>

#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

void visGridPath(const std::vector<Eigen::Vector3d> & _path,ros::Publisher & _vis_pub);


#endif // VISUALIZE_H