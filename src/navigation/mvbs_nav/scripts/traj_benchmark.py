import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.distance import cdist

# ========== 函数定义 ==========
def point_to_segment_distance(p, a, b):
    ap = p - a
    ab = b - a
    ab_norm = np.dot(ab, ab)
    if ab_norm == 0:
        return np.linalg.norm(ap)
    t = np.clip(np.dot(ap, ab) / ab_norm, 0, 1)
    projection = a + t * ab
    return np.linalg.norm(p - projection)

def trajectory_deviation_to_polyline(robot_coords, global_coords):
    deviations = []
    for p in robot_coords:
        min_dist = float('inf')
        for i in range(len(global_coords) - 1):
            a = global_coords[i]
            b = global_coords[i + 1]
            dist = point_to_segment_distance(p, a, b)
            min_dist = min(min_dist, dist)
        deviations.append(min_dist)
    return np.array(deviations)

def compute_path_length(path):
    diffs = np.diff(path, axis=0)
    segment_lengths = np.linalg.norm(diffs, axis=1)
    return np.sum(segment_lengths)

# ========== 1. 加载数据 ==========
global_path = pd.read_csv("data/waypoints/global_traj.csv")
robot_path = pd.read_csv("data/waypoints/DWA_2.csv")

global_coords = global_path[['position_x', 'position_y']].values
robot_coords = robot_path[['position_x', 'position_y']].values
time = robot_path['elapsed_time'].values
vx = robot_path['linear_vel_x'].values
vy = robot_path['linear_vel_y'].values

speed = np.sqrt(vx**2 + vy**2)
accel = np.diff(speed) / np.diff(time)
time_accel = time[:-1]

# ========== 2. 偏差计算 ==========
min_dists = trajectory_deviation_to_polyline(robot_coords, global_coords)
max_dev = np.max(min_dists)
min_dev = np.min(min_dists)
mean_dev = np.mean(min_dists)
rmse_dev = np.sqrt(np.mean(min_dists**2))

# ========== 3. 路径长度计算 ==========
global_length = compute_path_length(global_coords)
robot_length = compute_path_length(robot_coords)

# ========== 4. 加速度均方差 ==========
velocities = robot_path[['linear_vel_x', 'linear_vel_y']].values
accelerations = np.diff(velocities, axis=0)
accel_magnitudes = np.linalg.norm(accelerations, axis=1)
accel_variance = np.var(accel_magnitudes)

# ========== 5. 震荡估计 ==========
oscillations = np.sum(np.diff(np.sign(velocities[:, 0])) != 0) + \
               np.sum(np.diff(np.sign(velocities[:, 1])) != 0)

# ========== 6. 可视化 ==========
fig, axs = plt.subplots(2, 2, figsize=(12, 10))

# 图1：XY轨迹
axs[0, 0].plot(global_coords[:, 0], global_coords[:, 1], label='Global Path', color='blue', linewidth=2)
axs[0, 0].plot(robot_coords[:, 0], robot_coords[:, 1], label='Robot Path', color='red', linestyle='--', linewidth=1.5)
axs[0, 0].set_title("XY Trajectory")
axs[0, 0].set_xlabel("X (m)")
axs[0, 0].set_ylabel("Y (m)")
axs[0, 0].legend()
axs[0, 0].axis("equal")
axs[0, 0].grid(True)

# 图2：X vs 时间
axs[0, 1].plot(time, robot_coords[:, 0], label='Robot X', color='red')
axs[0, 1].plot(np.linspace(time[0], time[-1], len(global_coords)), global_coords[:, 0], label='Global X', color='blue')
axs[0, 1].set_title("X Position over Time")
axs[0, 1].set_xlabel("Time (s)")
axs[0, 1].set_ylabel("X (m)")
axs[0, 1].legend()
axs[0, 1].grid(True)

# 图3：Y vs 时间
axs[1, 0].plot(time, robot_coords[:, 1], label='Robot Y', color='red')
axs[1, 0].plot(np.linspace(time[0], time[-1], len(global_coords)), global_coords[:, 1], label='Global Y', color='blue')
axs[1, 0].set_title("Y Position over Time")
axs[1, 0].set_xlabel("Time (s)")
axs[1, 0].set_ylabel("Y (m)")
axs[1, 0].legend()
axs[1, 0].grid(True)

# 图4：速度 vs 时间
axs[1, 1].plot(time, speed, label='Speed', color='green')
axs[1, 1].set_title("Speed over Time")
axs[1, 1].set_xlabel("Time (s)")
axs[1, 1].set_ylabel("Speed (m/s)")
axs[1, 1].legend()
axs[1, 1].grid(True)

plt.tight_layout()

# ========== 图5：位置 + yaw ==========
qz = robot_path['orientation_z'].values
qw = robot_path['orientation_w'].values
yaw = 2 * np.arctan2(qz, qw)

fig1, axs1 = plt.subplots(2, 1, figsize=(10, 8), sharex=True)
axs1[0].plot(time, robot_coords[:, 0], label='Position X')
axs1[0].plot(time, robot_coords[:, 1], label='Position Y')
axs1[0].set_ylabel('Position (m)')
axs1[0].set_title('Position over Time')
axs1[0].legend()
axs1[0].grid(True)

axs1[1].plot(time, yaw, label='Yaw (rad)', color='purple')
axs1[1].set_xlabel('Time (s)')
axs1[1].set_ylabel('Yaw (rad)')
axs1[1].set_title('Yaw over Time')
axs1[1].legend()
axs1[1].grid(True)

fig1.tight_layout()

# ========== 图6：线速度 vx, vy ==========
fig2, ax2 = plt.subplots(figsize=(10, 4))
ax2.plot(time, vx, label='Linear Velocity X')
ax2.plot(time, vy, label='Linear Velocity Y')
ax2.set_xlabel('Time (s)')
ax2.set_ylabel('Velocity (m/s)')
ax2.set_title('Linear Velocities over Time')
ax2.legend()
ax2.grid(True)
fig2.tight_layout()

# 显示所有图
plt.show()

# ========== 7. 输出评估结果 ==========
print("===== 路径偏差评估指标 =====")
print(f"最大偏差: {max_dev:.4f} m")
print(f"最小偏差: {min_dev:.4f} m")
print(f"平均偏差: {mean_dev:.4f} m")
print(f"RMSE:     {rmse_dev:.4f} m\n")

print("===== 路径长度对比 =====")
print(f"全局轨迹长度: {global_length:.4f} m")
print(f"机器人实际路径长度: {robot_length:.4f} m\n")

print("===== 动态行为评估 =====")
print(f"加速度均方差: {accel_variance:.6f}")
print(f"估算震荡次数: {oscillations}")
