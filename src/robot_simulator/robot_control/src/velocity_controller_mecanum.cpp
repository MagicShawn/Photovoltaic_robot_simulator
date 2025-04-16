#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>
#include <vector>

class VelocityController
{
public:
    VelocityController() : nh_("~")
    {
        loadParameters();
        initPublishers();
        initSubscribers();
    }

private:
    void loadParameters()
    {
        // 加载并检查机器人参数
        nh_.param("wheel_radius", wheel_radius_, 0.2);
        nh_.param("wheel_separation", wheel_separation_, 0.78);
        nh_.param("max_linear_vel", max_linear_vel_, 3.0);
        nh_.param("max_angular_vel", max_angular_vel_, 2.0);

        // 参数有效性检查
        if (wheel_radius_ <= 0) {
            ROS_WARN("Invalid wheel_radius (%f), using default 0.2", wheel_radius_);
            wheel_radius_ = 0.2;
        }
        if (wheel_separation_ <= 0) {
            ROS_WARN("Invalid wheel_separation (%f), using default 0.78", wheel_separation_);
            wheel_separation_ = 0.78;
        }
    }

    void initPublishers()
    {
        // 初始化左右轮发布器组
        initPublisherGroup(left_pubs_, {
            "/back_left_velocity_controller/command",
            "/front_left_velocity_controller/command"
        });
        initPublisherGroup(right_pubs_, {
            "/back_right_velocity_controller/command",
            "/front_right_velocity_controller/command"
        });
    }

    void initPublisherGroup(std::vector<ros::Publisher>& pubs, const std::vector<std::string>& topics)
    {
        for (const auto& topic : topics) {
            pubs.push_back(nh_.advertise<std_msgs::Float64>(topic, 10));
        }
    }

    void initSubscribers()
    {
        cmd_vel_sub_ = nh_.subscribe("/cmd_vel", 10, &VelocityController::cmdVelCallback, this);
    }

    void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg)
    {
        // 带符号限制的速率的限幅处理
        const double v = clamp(msg->linear.x, -max_linear_vel_, max_linear_vel_);
        const double omega = clamp(msg->angular.z, -max_angular_vel_, max_angular_vel_);

        // 差速驱动运动学计算
        const double linear_term = v / wheel_radius_;
        const double angular_term = (omega * wheel_separation_) / (2.0 * wheel_radius_);
        
        // 构造左右轮速消息
        const std_msgs::Float64 left_msg = createSpeedMessage(linear_term - angular_term);
        const std_msgs::Float64 right_msg = createSpeedMessage(linear_term + angular_term);

        // 批量发布控制指令
        publishToGroup(left_pubs_, left_msg);
        publishToGroup(right_pubs_, right_msg);
    }

    std_msgs::Float64 createSpeedMessage(double speed) const
    {
        std_msgs::Float64 msg;
        msg.data = speed;
        return msg;
    }

    void publishToGroup(const std::vector<ros::Publisher>& pubs, const std_msgs::Float64& msg)
    {
        for (const auto& pub : pubs) {
            pub.publish(msg);
        }
    }

    double clamp(double value, double min, double max) const
    {
        return std::min(std::max(value, min), max);
    }

    ros::NodeHandle nh_;
    std::vector<ros::Publisher> left_pubs_;
    std::vector<ros::Publisher> right_pubs_;
    ros::Subscriber cmd_vel_sub_;

    // 机器人参数
    double wheel_radius_;
    double wheel_separation_;
    
    // 速度限制参数
    double max_linear_vel_;
    double max_angular_vel_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "velocity_controller");
    VelocityController controller;
    ros::spin();
    return 0;
}