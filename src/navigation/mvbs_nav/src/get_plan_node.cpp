#include <ros/ros.h>
#include <nav_msgs/GetPlan.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

std::vector<geometry_msgs::PoseStamped> waypoints;

std::vector<geometry_msgs::PoseStamped> loadWaypoints(ros::NodeHandle& nh) {
    std::vector<geometry_msgs::PoseStamped> waypoints;
    XmlRpc::XmlRpcValue waypoint_list;

    if (!nh.getParam("/waypoint_path_planner/waypoints", waypoint_list)) {
        ROS_ERROR("Failed to get 'waypoints' parameter");
        return waypoints;
    }

    if (waypoint_list.getType() != XmlRpc::XmlRpcValue::TypeArray) {
        ROS_ERROR("'waypoints' is not an array");
        return waypoints;
    }

    for (int i = 0; i < waypoint_list.size(); ++i) {
        if (waypoint_list[i].getType() != XmlRpc::XmlRpcValue::TypeArray || waypoint_list[i].size() < 3) {
            ROS_WARN("Skipping invalid waypoint at index %d", i);
            continue;
        }

        double x = static_cast<double>(waypoint_list[i][0]);
        double y = static_cast<double>(waypoint_list[i][1]);
        double yaw = static_cast<double>(waypoint_list[i][2]);

        geometry_msgs::PoseStamped pose;
        pose.header.frame_id = "map";
        pose.pose.position.x = x;
        pose.pose.position.y = y;

        tf2::Quaternion q;
        q.setRPY(0, 0, yaw);
        pose.pose.orientation = tf2::toMsg(q);

        waypoints.push_back(pose);
    }

    return waypoints;
}

nav_msgs::Path full_path;

bool generateFullPlan(ros::NodeHandle& nh) {
    ros::ServiceClient client = nh.serviceClient<nav_msgs::GetPlan>("/move_base/make_plan");
    nav_msgs::GetPlan srv;

    if (waypoints.size() < 2) {
        ROS_ERROR("Need at least two waypoints to generate a path.");
        return false;
    }

    full_path.poses.clear();
    full_path.header.frame_id = "map";
    full_path.header.stamp = ros::Time::now();

    for (size_t i = 0; i < waypoints.size() - 1; ++i) {
        srv.request.start = waypoints[i];
        srv.request.goal = waypoints[i + 1];
        srv.request.tolerance = 0.1;

        if (client.call(srv) && !srv.response.plan.poses.empty()) {
            full_path.poses.insert(full_path.poses.end(),
                                   srv.response.plan.poses.begin(),
                                   srv.response.plan.poses.end());
        } else {
            ROS_WARN("Failed to get plan between waypoint %lu and %lu", i, i + 1);
        }
    }

    return true;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "waypoint_path_planner");
    ros::NodeHandle nh;

    ros::Publisher path_pub = nh.advertise<nav_msgs::Path>("waypoint_path", 1, true);
    waypoints = loadWaypoints(nh);

    // 先生成一次路径
    if (!generateFullPlan(nh)) {
        ROS_ERROR("Failed to generate full path.");
        return 1;
    }

    

    ros::Rate loop_rate(1);  // 每秒发布一次
    while (ros::ok()) {
        full_path.header.stamp = ros::Time::now();  // 更新时间戳
        path_pub.publish(full_path);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
