#include "my_Astar.h"
#include "visualize.h"

bool has_map = false;

double min_cord_x,min_cord_y,resolution;
int width,height;

Eigen::Vector3d st_pt;

my_AstarPlanner * astarPlanner = new my_AstarPlanner();

ros::Publisher vis_pub;

//multiplanning
int goalpoints_Num = 8;
vector<Eigen::Vector3d> goalpointsArray;
bool is_planning = false;
std::mutex goalpoints_mutex;

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
    ROS_WARN("[astar_node] there are %d points in the cloud" , (int)cloud.points.size());
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
    Eigen::Vector3d new_point(goal.pose.position.x, goal.pose.position.y, 0.0);

    {
        std::lock_guard<std::mutex> lock(goalpoints_mutex);
        if(goalpointsArray.size() >= goalpoints_Num) {
            goalpointsArray.clear(); // 超过5个点则清空重置
        }
        goalpointsArray.push_back(new_point);
        ROS_WARN("[astar_node] received No %ld goal points ",
            goalpointsArray.size());
    }

    if(goalpointsArray.size() == goalpoints_Num) {
        is_planning = true;
    }

}

void multipathPlanning(){
    vector<Eigen::Vector3d> path_segments;

    {
        std::lock_guard<std::mutex> lock(goalpoints_mutex);
        if(goalpointsArray.size() != goalpoints_Num) return;

        // 生成连续路径段
        for(size_t i = 0; i < goalpointsArray.size()-1; i++) {
            auto path = astarPlanner->pathFinding(goalpointsArray[i], goalpointsArray[i+1]);
            if(!path.empty()) {
                path_segments.insert(path_segments.end(), path.begin(), path.end()); // 避免重复点
            }
        }
        goalpointsArray.clear();
    }

    // 发布合并后的路径
    if(!path_segments.empty()) {
        visGridPath(path_segments, vis_pub);
    }

    is_planning = false;
}

int main(int argc , char* argv[]){

    ros::init(argc,argv,"custom_master");
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

    ros::Rate rate(10);
    while(ros::ok()) 
    {
        ros::spinOnce();

        // 定时检查并执行多段规划
        static ros::Time last_plan_time = ros::Time::now();
        if(is_planning && (ros::Time::now() - last_plan_time > ros::Duration(0.5))) {
            ROS_WARN("[astar_node] received 8 goal points, start planning...");
            multipathPlanning();
            last_plan_time = ros::Time::now();
        }

        rate.sleep();
    }

    delete astarPlanner;

    return 0 ;
}