import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

# ========= 函数定义 =========
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

# ========= 文件路径定义 =========
global_files = [
    "data/waypoints/sim_test/global_traj_3.csv",
    "data/waypoints/sim_test/global_traj_3.csv",
    "data/waypoints/sim_test/global_traj_3.csv"
]

robot_files = [
    "data/waypoints/sim_test/3_1.csv",
    "data/waypoints/sim_test/3_2.csv",
    "data/waypoints/sim_test/3_3.csv"
]

colors = ['red', 'green', 'orange']
linestyles = ['--', '-.', ':']
results = []

# ========= 图像初始化 =========
fig, axs = plt.subplots(2, 2, figsize=(12, 10))

# ========= 主循环处理每组路径 =========
for i in range(3):
    global_path = pd.read_csv(global_files[i])
    robot_path = pd.read_csv(robot_files[i])

    global_coords = global_path[['x', 'y']].values
    robot_coords = robot_path[['position_x', 'position_y']].values
    time = robot_path['elapsed_time'].values
    vx = robot_path['linear_vel_x'].values
    vy = robot_path['linear_vel_y'].values
    speed = np.sqrt(vx**2 + vy**2)

    # 偏差计算
    min_dists = trajectory_deviation_to_polyline(robot_coords, global_coords)
    max_dev = np.max(min_dists)
    min_dev = np.min(min_dists)
    mean_dev = np.mean(min_dists)
    rmse_dev = np.sqrt(np.mean(min_dists**2))

    # 路径长度
    global_length = compute_path_length(global_coords)
    robot_length = compute_path_length(robot_coords)

    # 加速度与震荡估计
    velocities = robot_path[['linear_vel_x', 'linear_vel_y']].values
    accelerations = np.diff(velocities, axis=0)
    accel_magnitudes = np.linalg.norm(accelerations, axis=1)
    accel_variance = np.var(accel_magnitudes)
    oscillations = np.sum(np.diff(np.sign(velocities[:, 0])) != 0) + \
                   np.sum(np.diff(np.sign(velocities[:, 1])) != 0)

    # 存储评估结果
    results.append({
        'max_dev': max_dev,
        'min_dev': min_dev,
        'mean_dev': mean_dev,
        'rmse_dev': rmse_dev,
        'global_length': global_length,
        'robot_length': robot_length,
        'accel_var': accel_variance,
        'oscillations': oscillations
    })

    # ========== 图像绘制 ==========
    axs[0, 0].plot(global_coords[:, 0], global_coords[:, 1],
                   label=f'Global Path {i+1}', color=colors[i], linewidth=2)
    axs[0, 0].plot(robot_coords[:, 0], robot_coords[:, 1],
                   label=f'Robot Path {i+1}', color=colors[i], linestyle=linestyles[i], linewidth=1.5)

    axs[0, 1].plot(time, robot_coords[:, 0], label=f'Robot X {i+1}', color=colors[i], linestyle=linestyles[i])
    axs[0, 1].plot(np.linspace(time[0], time[-1], len(global_coords)), global_coords[:, 0],
                   label=f'Global X {i+1}', color=colors[i])

    axs[1, 0].plot(time, robot_coords[:, 1], label=f'Robot Y {i+1}', color=colors[i], linestyle=linestyles[i])
    axs[1, 0].plot(np.linspace(time[0], time[-1], len(global_coords)), global_coords[:, 1],
                   label=f'Global Y {i+1}', color=colors[i])

    axs[1, 1].plot(time, speed, label=f'Speed {i+1}', color=colors[i])

# ========= 图像整理 =========
axs[0, 0].set_title("XY Trajectory")
axs[0, 0].set_xlabel("X (m)")
axs[0, 0].set_ylabel("Y (m)")
axs[0, 0].axis("equal")
axs[0, 0].legend()
axs[0, 0].grid(True)

axs[0, 1].set_title("X Position over Time")
axs[0, 1].set_xlabel("Time (s)")
axs[0, 1].set_ylabel("X (m)")
axs[0, 1].legend()
axs[0, 1].grid(True)

axs[1, 0].set_title("Y Position over Time")
axs[1, 0].set_xlabel("Time (s)")
axs[1, 0].set_ylabel("Y (m)")
axs[1, 0].legend()
axs[1, 0].grid(True)

axs[1, 1].set_title("Speed over Time")
axs[1, 1].set_xlabel("Time (s)")
axs[1, 1].set_ylabel("Speed (m/s)")
axs[1, 1].legend()
axs[1, 1].grid(True)

plt.tight_layout()
plt.show()

# ========= 结果输出 =========
for i, res in enumerate(results):
    print(f"===== 路径评估：方案 {i+1} =====")
    print(f"最大偏差: {res['max_dev']:.4f} m")
    print(f"最小偏差: {res['min_dev']:.4f} m")
    print(f"平均偏差: {res['mean_dev']:.4f} m")
    print(f"RMSE:     {res['rmse_dev']:.4f} m")
    print(f"全局轨迹长度: {res['global_length']:.4f} m")
    print(f"机器人路径长度: {res['robot_length']:.4f} m")
    print(f"加速度均方差: {res['accel_var']:.6f}")
    print(f"估算震荡次数: {res['oscillations']}\n")
