#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <termios.h>
#include <unistd.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "keyboard_teleop");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    // 保存旧的终端设置
    struct termios old_tio, new_tio;
    tcgetattr(STDIN_FILENO, &old_tio);
    new_tio = old_tio;

    // 更改进终端模式：非规范模式，关闭回显
    new_tio.c_lflag &= ~(ICANON | ECHO);
    new_tio.c_cc[VMIN] = 0;
    new_tio.c_cc[VTIME] = 0;
    tcsetattr(STDIN_FILENO, TCSANOW, &new_tio);

    double speed_factor = 0.5;
    double lx = 0.0, ly = 0.0, az = 0.0;
    char c;

    geometry_msgs::Twist twist;
    ROS_INFO("Use WASD/QEZC/JL to control, ,/. to adjust speed. Press 's' to stop.");

    ros::Rate loop_rate(10); // 10Hz
    while (ros::ok()) {
        if (read(STDIN_FILENO, &c, 1) > 0) {
            switch(c) {
                case 'w': lx = 1.0; ly = 0.0; az = 0.0; break;
                case 'x': lx = -1.0; ly = 0.0; az = 0.0; break;
                case 'a': ly = 1.0; lx = 0.0; az = 0.0; break;
                case 'd': ly = -1.0; lx = 0.0; az = 0.0; break;
                case 'q': lx = 1.0; ly = 1.0; az = 0.0; break;
                case 'e': lx = 1.0; ly = -1.0; az = 0.0; break;
                case 'z': lx = -1.0; ly = 1.0; az = 0.0; break;
                case 'c': lx = -1.0; ly = -1.0; az = 0.0; break;
                case 'j': az = 3.0; lx = 0.0; ly = 0.0; break;
                case 'l': az = -3.0; lx = 0.0; ly = 0.0; break;
                case 's': lx = 0.0; ly = 0.0; az = 0.0; break;
                case ',': speed_factor = std::max(0.1, speed_factor - 0.1); break;
                case '.': speed_factor = std::min(2.0, speed_factor + 0.1); break;
                default: break;
            }
            // 更新速度
            twist.linear.x = lx * speed_factor;
            twist.linear.y = ly * speed_factor;
            twist.angular.z = az * speed_factor;
        }

        pub.publish(twist);
        loop_rate.sleep();
    }

    // 恢复终端设置
    tcsetattr(STDIN_FILENO, TCSANOW, &old_tio);
    return 0;
}