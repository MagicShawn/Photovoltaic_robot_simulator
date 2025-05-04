#!/usr/bin/env python3
import rospy
import tf
import numpy as np
import matplotlib.pyplot as plt
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist, TwistStamped
from threading import Lock
import math
import signal
import sys

# 全局变量
global_plan = []
amcl_traj = []
odom_traj = []
odom_yaw = []

cmd_vel_data = []
cmd_times = []
start_time = None

lock = Lock()

def global_plan_cb(msg):
    global global_plan
    with lock:
        if not global_plan:
            for pose in msg.poses:
                global_plan.append((pose.pose.position.x, pose.pose.position.y))

def amcl_cb(msg):
    with lock:
        amcl_traj.append((msg.pose.pose.position.x, msg.pose.pose.position.y))

def odom_cb(msg):
    with lock:
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        quat = msg.pose.pose.orientation
        _, _, yaw = tf.transformations.euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
        odom_traj.append((x, y))
        odom_yaw.append(yaw)

def cmd_cb(msg):
    global start_time
    with lock:
        t = rospy.get_time()
        if start_time is None:
            start_time = t
        cmd_times.append(t - start_time)
        try:
            vx = msg.linear.x
            vy = msg.linear.y
        except:
            vx, vy = 0.0, 0.0
        cmd_vel_data.append((vx, vy, msg.angular.z))

def path_length(points):
    return sum(math.hypot(x2 - x1, y2 - y1) for (x1, y1), (x2, y2) in zip(points[:-1], points[1:]))

def compute_deviation(reference_path, actual_path):
    deviations = []
    for ax, ay in actual_path:
        dists = [math.hypot(ax - rx, ay - ry) for rx, ry in reference_path]
        deviations.append(min(dists))
    return deviations

def velocity_smoothness(velocities):
    velocities = np.array(velocities)
    diffs = np.diff(velocities, axis=0)
    return np.linalg.norm(diffs, axis=1).mean()

def plot_results():
    with lock:
        fig, axs = plt.subplots(3, 1, figsize=(12, 16))

        # --- 图1: 平面轨迹图 ---
        axs[0].set_title("Trajectory (Global Plan, AMCL, Odom)")
        if global_plan:
            plan_np = np.array(global_plan)
            axs[0].plot(plan_np[:, 0], plan_np[:, 1], 'g--', label='Global Plan')
        if amcl_traj:
            amcl_np = np.array(amcl_traj)
            axs[0].plot(amcl_np[:, 0], amcl_np[:, 1], 'b-', label='AMCL')
        if odom_traj:
            odom_np = np.array(odom_traj)
            axs[0].plot(odom_np[:, 0], odom_np[:, 1], 'r-', label='Odom')
        axs[0].legend()
        axs[0].axis('equal')
        axs[0].grid()

        # --- 图2: 位置随时间变化 ---
        axs[1].set_title("X / Y with time (Odom)")
        if odom_traj and start_time is not None:
            odom_np = np.array(odom_traj)
            dt = 0.05  # 估计采样间隔
            t = np.linspace(0, dt * len(odom_np), len(odom_np))
            axs[1].plot(t, odom_np[:, 0], 'r-', label='X')
            axs[1].plot(t, odom_np[:, 1], 'b-', label='Y')
            axs[1].legend()
            axs[1].grid()

        # --- 图3: 速度图 ---
        axs[2].set_title("Velocity (Vx, Vy, Yaw Rate)")
        if cmd_vel_data and cmd_times:
            cmd_np = np.array(cmd_vel_data)
            axs[2].plot(cmd_times, cmd_np[:, 0], 'b-', label='Vx')
            axs[2].plot(cmd_times, cmd_np[:, 1], 'r-', label='Vy')
            axs[2].plot(cmd_times, cmd_np[:, 2], 'g--', label='Yaw Rate')
            axs[2].legend()
            axs[2].grid()

        plt.tight_layout()
        plt.show()

        # === 数值分析 ===
        print("\n===== 统计结果 =====")
        if global_plan and amcl_traj:
            deviations = compute_deviation(global_plan, amcl_traj)
            print(f"平均轨迹偏差: {np.mean(deviations):.3f} m")
            print(f"最大轨迹偏差: {np.max(deviations):.3f} m")
        if global_plan:
            print(f"全局路径长度: {path_length(global_plan):.2f} m")
        if amcl_traj:
            print(f"AMCL轨迹长度: {path_length(amcl_traj):.2f} m")
        if odom_traj:
            print(f"Odom轨迹长度: {path_length(odom_traj):.2f} m")
        if cmd_vel_data:
            print(f"速度平滑性（越小越平稳）: {velocity_smoothness(cmd_vel_data):.4f}")

def signal_handler(sig, frame):
    print("\n捕获Ctrl+C,绘图并退出...")
    plot_results()
    sys.exit(0)

def main():
    signal.signal(signal.SIGINT, signal_handler)
    rospy.init_node("realscene_data_plotter", anonymous=True)

    rospy.Subscriber("/move_base/GlobalPlanner/plan", Path, global_plan_cb)
    rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, amcl_cb)
    rospy.Subscriber("/odom", Odometry, odom_cb)
    rospy.Subscriber("/cmd_vel", Twist, cmd_cb)

    print("正在监听,请播放你关心的bag时间片段...")
    rospy.spin()

if __name__ == '__main__':
    main()
