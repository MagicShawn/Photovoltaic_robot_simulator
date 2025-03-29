#include "my_Astar.h"
#include "visualize.h"

bool has_map = false;

double min_cord_x,min_cord_y,resolution;
int width,height;

Eigen::Vector3d st_pt;

my_AstarPlanner * astarPlanner = new my_AstarPlanner();

ros::Publisher vis_pub;

/// @brief receive pointcloud call back
/// @param pointcloud_map 
void rcvPointCloudCallBack(const sensor_msgs::PointCloud2 & pointcloud_map)
{   
    if(has_map){return;}
    ROS_WARN("[astar_node] received a map !");
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::PointCloud<pcl::PointXYZ> cloud_vis;
    sensor_msgs::PointCloud2 map_vis;

    pcl::fromROSMsg(pointcloud_map, cloud);
    ROS_WARN("[astar_node] there are %ld points in the cloud" , (int)cloud.points.size());
    if((int)cloud.points.size() == 0){return;}

    pcl::PointXYZ pt;
    for(int idx=0;idx < (int)cloud.points.size();idx++){
        pt = cloud.points[idx];
        // ROS_INFO("[astar_node] obs x: %.3f y: %.3f z: %.3f",pt.x,pt.y,pt.z);
        //set obstacles
        astarPlanner->setObs(Eigen::Vector3d(pt.x,pt.y,pt.z));
    }

    has_map = true;

}

void rcvGoalPointCallBack(const geometry_msgs::PoseStamped & goal){
    ROS_WARN("[astar_node] received a goal !");
    double goal_x = goal.pose.position.x;
    double goal_y = goal.pose.position.y;
    Eigen::Vector3d goalCord = Eigen::Vector3d(goal_x,goal_y,0.0);

    auto grid_path = astarPlanner->pathFinding(st_pt,goalCord);
    visGridPath(grid_path,vis_pub);

}



int main(int argc , char* argv[]){

    ros::init(argc,argv,"master");
    ros::NodeHandle nh("~");

    nh.param("map/min_cord_x",min_cord_x,-50.0);
    nh.param("map/min_cord_y",min_cord_y,-50.0);
    nh.param("map/map_width",width,0);
    nh.param("map/map_height",height,0);
    nh.param("map/resolution",resolution,0.025);

    nh.param("planning/st_cord_x",st_pt.x(),0.0);
    nh.param("planning/st_cord_y",st_pt.y(),0.0);
    nh.param("planning/st_cord_z",st_pt.z(),0.0);

    if(!astarPlanner->init(resolution,min_cord_x,min_cord_y,width,height)){
        ros::shutdown();
    }

    vis_pub = nh.advertise<sensor_msgs::PointCloud2>("vis_grid_path",10);
    ros::Subscriber map_sub = nh.subscribe("map",1,rcvPointCloudCallBack);
    ros::Subscriber goal_sub = nh.subscribe("goal",1,rcvGoalPointCallBack);

    ros::Rate rate(100);
    bool status = ros::ok();
    while(status) 
    {
        ros::spinOnce();      
        status = ros::ok();
        rate.sleep();
    }

    delete astarPlanner;

    return 0 ;
}