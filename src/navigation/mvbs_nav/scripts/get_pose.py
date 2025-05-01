#!/usr/bin/env python
import rospy
import csv
import os
from datetime import datetime
from gazebo_msgs.msg import ModelStates

class RobotStateLogger:
    def __init__(self):
        self.robot_position = None
        self.robot_orientation = None
        self.linear_velocity = None
        self.angular_velocity = None
        self.start_time = None  # 记录采样开始时间

        # 初始化文件存储
        self.data_file = None
        self.csv_writer = None
        self.init_data_file()

        # 订阅 Gazebo 中模型状态信息
        rospy.Subscriber('/gazebo/model_states', ModelStates, self.model_states_callback)

        # 创建定时器 (10Hz)
        rospy.Timer(rospy.Duration(0.1), self.timer_callback)

        # 注册关闭回调
        rospy.on_shutdown(self.on_shutdown)

    def init_data_file(self):
        file_dir = os.path.expanduser("/home/maxliang/Project/Mine/BiShe/Reference/Photovoltaic_robot_simulator/data/")
        if not os.path.exists(file_dir):
            os.makedirs(file_dir)

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = os.path.join(file_dir, f"robot_state_{timestamp}.csv")

        try:
            self.data_file = open(filename, 'w')
            self.csv_writer = csv.writer(self.data_file)
            self.csv_writer.writerow([
                'elapsed_time',
                'position_x', 'position_y', 'orientation_z', 'orientation_w',
                'linear_vel_x', 'linear_vel_y', 'angular_vel_z'  # 新增字段
            ])
            rospy.loginfo(f"数据文件已创建: {filename}")
        except Exception as e:
            rospy.logerr(f"文件创建失败: {str(e)}")
            rospy.signal_shutdown("文件错误")

    def model_states_callback(self, msg):
        try:
            robot_index = msg.name.index('robot') # waypoints:robot traj:solar_car
            self.robot_position = msg.pose[robot_index].position
            self.robot_orientation = msg.pose[robot_index].orientation
            self.linear_velocity = msg.twist[robot_index].linear
            self.angular_velocity = msg.twist[robot_index].angular
        except ValueError:
            pass

    def timer_callback(self, event):
        if (self.robot_position and self.robot_orientation and
                self.linear_velocity and self.angular_velocity):
            
            current_time = rospy.Time.now()
            if self.start_time is None:
                self.start_time = current_time
                elapsed_time = 0.0
            else:
                elapsed_time = round((current_time - self.start_time).to_sec(), 4)

            self.csv_writer.writerow([
                elapsed_time,
                round(self.robot_position.x, 4),
                round(self.robot_position.y, 4),
                # round(self.robot_position.z, 4),
                # round(self.robot_orientation.x, 4),
                # round(self.robot_orientation.y, 4),
                round(self.robot_orientation.z, 4),
                round(self.robot_orientation.w, 4),
                round(self.linear_velocity.x, 4),    # 新增线速度 x
                round(self.linear_velocity.y, 4),    # 新增线速度 y
                round(self.angular_velocity.z, 4)    # 新增角速度 z
            ])

            rospy.loginfo("[%.1fs] Pos:(%.2f, %.2f) Yaw:(%.2f) | Vel:(%.2f, %.2f) | YawRate: %.2f",
                          elapsed_time,
                          self.robot_position.x, self.robot_position.y,
                          self.robot_orientation.z,
                          self.linear_velocity.x, self.linear_velocity.y,
                          self.angular_velocity.z)

    def on_shutdown(self):
        if self.data_file:
            self.data_file.close()
            rospy.loginfo("数据文件已关闭")

if __name__ == '__main__':
    rospy.init_node('robot_state_logger')
    RobotStateLogger()
    rospy.spin()
