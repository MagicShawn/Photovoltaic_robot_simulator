#!/usr/bin/env python3
import rospy
import csv
import os
from nav_msgs.msg import Odometry

odom_data = []
t0 = None

# 设置输出路径（可自行修改）
output_path = os.path.expanduser("data/waypoints/real_test/trajectory_data.csv")

def odom_cb(msg):
    global t0

    t = msg.header.stamp.to_sec()
    if t0 is None:
        t0 = t
    elapsed = t - t0

    pos = msg.pose.pose.position
    ori = msg.pose.pose.orientation
    twist = msg.twist.twist

    odom_data.append([
        f"{elapsed:.3f}",
        f"{pos.x:.4f}",
        f"{pos.y:.4f}",
        f"{ori.z:.4f}",
        f"{ori.w:.4f}",
        f"{twist.linear.x:.4f}",
        f"{twist.linear.y:.4f}",
        f"{twist.angular.z:.4f}",
    ])

def write_csv():
    if not odom_data:
        rospy.logwarn("No odom data collected, CSV not written.")
        return

    with open(output_path, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow([
            "elapsed_time",
            "position_x",
            "position_y",
            "orientation_z",
            "orientation_w",
            "linear_vel_x",
            "linear_vel_y",
            "angular_vel_z"
        ])
        writer.writerows(odom_data)

    rospy.loginfo(f"Saved odom data to: {output_path}")

def main():
    rospy.init_node("odom_logger")
    rospy.Subscriber("/odom", Odometry, odom_cb)
    rospy.on_shutdown(write_csv)
    rospy.loginfo("Listening to /odom. Press Ctrl+C to stop and save.")
    rospy.spin()

if __name__ == "__main__":
    main()
