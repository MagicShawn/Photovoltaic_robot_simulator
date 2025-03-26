#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>

class VelocityController
{
public:
    VelocityController() : nh_("~")
    {
        // 初始化发布器
        back_right_pub_ = nh_.advertise<std_msgs::Float64>("/back_right_velocity_controller/command", 10);
        back_left_pub_ = nh_.advertise<std_msgs::Float64>("/back_left_velocity_controller/command", 10);
        front_right_pub_ = nh_.advertise<std_msgs::Float64>("/front_right_velocity_controller/command", 10);
        front_left_pub_ = nh_.advertise<std_msgs::Float64>("/front_left_velocity_controller/command", 10);

        // 订阅cmd_vel话题
        cmd_vel_sub_ = nh_.subscribe("/cmd_vel", 10, &VelocityController::cmdVelCallback, this);

        // 加载参数
        nh_.param("wheel_radius", wheel_radius_, 0.2);
        nh_.param("wheel_separation", wheel_separation_, 0.78);
        nh_.param("wheel_base", wheel_base_, 0.4);
    }

private:
    void cmdVelCallback(const geometry_msgs::Twist::ConstPtr &msg)
    {
        // 限幅处理
        double v = clamp(msg->linear.x, -max_linear_vel_, max_linear_vel_);
        double omega = clamp(msg->angular.z, -max_angular_vel_, max_angular_vel_);

        // 计算轮速（四轮差速模型）
        double left_vel = (v - omega * wheel_separation_ / 2) / wheel_radius_;
        double right_vel = (v + omega * wheel_separation_ / 2) / wheel_radius_;

        // 创建消息
        std_msgs::Float64 left_msg, right_msg;
        left_msg.data = left_vel;
        right_msg.data = right_vel;

        // 发布速度指令
        back_left_pub_.publish(left_msg);
        front_left_pub_.publish(left_msg);
        back_right_pub_.publish(right_msg);
        front_right_pub_.publish(right_msg);
    }

    double clamp(double value, double min, double max)
    {
        return value < min ? min : (value > max ? max : value);
    }

    ros::NodeHandle nh_;
    ros::Publisher back_right_pub_;
    ros::Publisher back_left_pub_;
    ros::Publisher front_right_pub_;
    ros::Publisher front_left_pub_;
    ros::Subscriber cmd_vel_sub_;

    // 机器人参数
    double wheel_radius_;
    double wheel_separation_;
    double wheel_base_;

    // 速度限制
    const double max_linear_vel_ = 3.0;  // m/s
    const double max_angular_vel_ = 2.0; // rad/s
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "velocity_controller");
    VelocityController controller;
    ros::spin();
    return 0;
}