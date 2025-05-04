import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import os

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

def plot_closest_points_deviation(global_coords, robot_coords, path_index, color='blue'):
    plt.figure(figsize=(8, 6))
    plt.plot(global_coords[:, 0], global_coords[:, 1], 'o-', label='Global Path', color=color)
    plt.plot(robot_coords[:, 0], robot_coords[:, 1], '-', label='Robot Path', color='gray', alpha=0.7)

    deviations = []
    for i, gp in enumerate(global_coords):
        dists = np.linalg.norm(robot_coords - gp, axis=1)
        min_idx = np.argmin(dists)
        closest_point = robot_coords[min_idx]
        deviation = dists[min_idx]
        deviations.append(deviation)

        # 画连线
        plt.plot([gp[0], closest_point[0]], [gp[1], closest_point[1]], 'k--', linewidth=0.8)
        plt.plot(closest_point[0], closest_point[1], 'x', color='red', markersize=5)
        plt.annotate(f"{deviation:.2f}m", xy=closest_point, textcoords="offset points", xytext=(5, 5), fontsize=8)

    plt.title(f"Closest Point Deviation - Path {path_index+1}")
    plt.xlabel("X (m)")
    plt.ylabel("Y (m)")
    plt.legend()
    plt.axis("equal")
    plt.grid(True)
    plt.tight_layout()
    plt.show()

    print(f"\n>>> 方案 {path_index+1} 的全局路标点与最近机器人轨迹点偏差：")
    for i, d in enumerate(deviations):
        print(f"路标点 {i+1}: 偏差 = {d:.4f} m")
    print(f"平均偏差: {np.mean(deviations):.4f} m\n")

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
    robot_path = pd.read_csv(robot_files[i])
    robot_coords = robot_path[['position_x', 'position_y']].values
    time = robot_path['elapsed_time'].values
    vx = robot_path['linear_vel_x'].values
    vy = robot_path['linear_vel_y'].values
    speed = np.sqrt(vx**2 + vy**2)

    # 默认值（无global path时）
    max_dev = min_dev = mean_dev = rmse_dev = global_length = None

    # 检查 global path 是否存在
    if os.path.exists(global_files[i]):
        global_path = pd.read_csv(global_files[i])
        global_coords = global_path[['x', 'y']].values

        # 偏差计算
        min_dists = trajectory_deviation_to_polyline(robot_coords, global_coords)
        max_dev = np.max(min_dists)
        min_dev = np.min(min_dists)
        mean_dev = np.mean(min_dists)
        rmse_dev = np.sqrt(np.mean(min_dists**2))

        # 路径长度
        global_length = compute_path_length(global_coords)


        # 轨迹图
        axs[0, 0].plot(global_coords[:, 0], global_coords[:, 1],
                       label=f'Global Path {i+1}', color=colors[i], linewidth=2)

        axs[0, 1].plot(np.linspace(time[0], time[-1], len(global_coords)), global_coords[:, 0],
                       label=f'Global X {i+1}', color=colors[i])
        axs[1, 0].plot(np.linspace(time[0], time[-1], len(global_coords)), global_coords[:, 1],
                       label=f'Global Y {i+1}', color=colors[i])

    # 机器人轨迹长度
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

    # 绘图：机器人轨迹 & 时间变化图
    axs[0, 0].plot(robot_coords[:, 0], robot_coords[:, 1],
                   label=f'Robot Path {i+1}', color=colors[i], linestyle=linestyles[i], linewidth=1.5)

    axs[0, 1].plot(time, robot_coords[:, 0], label=f'Robot X {i+1}', color=colors[i], linestyle=linestyles[i])
    axs[1, 0].plot(time, robot_coords[:, 1], label=f'Robot Y {i+1}', color=colors[i], linestyle=linestyles[i])
    axs[1, 1].plot(time, speed, label=f'Speed {i+1}', color=colors[i])

    ENABLE_CLOSEST_POINT_PLOT = True  # 改为 False 可禁用

    if ENABLE_CLOSEST_POINT_PLOT and global_length is not None:
        plot_closest_points_deviation(global_coords, robot_coords, i, color=colors[i])


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
    if res['max_dev'] is not None:
        print(f"最大偏差: {res['max_dev']:.4f} m")
        print(f"最小偏差: {res['min_dev']:.4f} m")
        print(f"平均偏差: {res['mean_dev']:.4f} m")
        print(f"RMSE:     {res['rmse_dev']:.4f} m")
        print(f"全局轨迹长度: {res['global_length']:.4f} m")
    else:
        print("无Global Path,跳过偏差与轨迹长度评估。")

    print(f"机器人路径长度: {res['robot_length']:.4f} m")
    print(f"加速度均方差: {res['accel_var']:.6f}")
    print(f"估算震荡次数: {res['oscillations']}\n")
