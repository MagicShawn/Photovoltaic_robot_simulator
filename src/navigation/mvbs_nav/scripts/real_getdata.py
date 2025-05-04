#!/usr/bin/env python3
import rospy
import csv
import os
import tf
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped

# ===== 保存目录 =====
output_dir = "data/waypoints/real_test"
os.makedirs(output_dir, exist_ok=True)

# ===== 全局变量 =====
global_plan_saved = False
amcl_data = []
odom_data = []
t0 = None

# ===== 回调函数 =====
def global_plan_cb(msg):
    global global_plan_saved
    if global_plan_saved:
        return
    global_plan_saved = True

    path = os.path.join(output_dir, "global_plan.csv")
    with open(path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["x", "y"])
        for pose in msg.poses:
            x = pose.pose.position.x
            y = pose.pose.position.y
            writer.writerow([f"{x:.6f}", f"{y:.6f}"])
    rospy.loginfo(f"全局路径已保存到 {path}")

def amcl_cb(msg):
    global t0
    t = msg.header.stamp.to_sec()
    if t0 is None:
        t0 = t
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    q = msg.pose.pose.orientation
    amcl_data.append((t, x, y, q.z, q.w))  # 用 orientation_z, orientation_w

def odom_cb(msg):
    global t0
    t = msg.header.stamp.to_sec()
    if t0 is None:
        t0 = t
    vx = msg.twist.twist.linear.x
    vy = msg.twist.twist.linear.y
    wz = msg.twist.twist.angular.z
    odom_data.append((t, vx, vy, wz))

# ===== 写入对齐的 CSV =====
def write_aligned_csv():
    if not amcl_data or not odom_data:
        rospy.logwarn("amcl或odom数据为空，无法保存对齐数据")
        return

    path = os.path.join(output_dir, "trajectory_data.csv")
    with open(path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow([
            "elapsed_time",
            "position_x", "position_y",
            "orientation_z", "orientation_w",
            "linear_vel_x", "linear_vel_y", "angular_vel_z"
        ])

        ai, oi = 0, 0
        while ai < len(amcl_data) and oi < len(odom_data):
            at, *_ = amcl_data[ai]
            ot, *_ = odom_data[oi]
            if abs(at - ot) < 0.05:
                elapsed = min(at, ot) - t0
                row = [elapsed] + list(amcl_data[ai][1:]) + list(odom_data[oi][1:])
                writer.writerow([f"{elapsed:.3f}"] + [f"{x:.4f}" for x in row[1:]])
                ai += 1
                oi += 1
            elif at < ot:
                ai += 1
            else:
                oi += 1
    rospy.loginfo(f"轨迹数据已保存到 {path}")

# ===== 主程序 =====
def main():
    rospy.init_node("data_logger")

    rospy.Subscriber("/move_base/GlobalPlanner/plan", Path, global_plan_cb)
    rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, amcl_cb)
    rospy.Subscriber("/odom", Odometry, odom_cb)

    rospy.loginfo("开始监听数据...")
    rospy.on_shutdown(write_aligned_csv)
    rospy.spin()

if __name__ == "__main__":
    main()
