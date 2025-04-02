#include "visualize.h"

void visGridPath(const std::vector<Eigen::Vector3d> & _path,ros::Publisher & _vis_pub){
    pcl::PointCloud<pcl::PointXYZ> path_cloud;
    sensor_msgs::PointCloud2 ros_path_cloud;

    for(auto it = _path.begin();it != _path.end();it++){
        pcl::PointXYZ pt;
        pt.x = it->x();
        pt.y = it->y();
        pt.z = it->z();

        path_cloud.points.push_back(pt);
    }

    path_cloud.width = path_cloud.points.size();
    path_cloud.height = 1;
    path_cloud.is_dense = true;
    pcl::toROSMsg(path_cloud,ros_path_cloud);
    ros_path_cloud.header.frame_id = "world";

    _vis_pub.publish(ros_path_cloud);
}