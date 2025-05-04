import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import math

robot_path="data/waypoints/real_test/trajectory_data_1_3.csv"
# 读取数据
df = pd.read_csv(robot_path)

# 时间序列
time = df['elapsed_time'].values
x = df['position_x'].values
y = df['position_y'].values

# 从四元数计算 z 轴欧拉角（yaw）
qz = df['orientation_z'].values
qw = df['orientation_w'].values
yaw = 2 * np.arctan2(qz, qw)  # 四元数转 yaw（假设 x=y=0）

vx = df['linear_vel_x'].values
vy = df['linear_vel_y'].values

# ========================
# 图 1: x,y 和 yaw 关于时间（2子图）
# ========================
fig1, axs = plt.subplots(2, 1, figsize=(10, 8), sharex=True)

axs[0].plot(time, x, label='Position X')
axs[0].plot(time, y, label='Position Y')
axs[0].set_ylabel('Position (m)')
axs[0].set_title('Position over Time')
axs[0].legend()
axs[0].grid(True)

axs[1].plot(time, yaw, label='Yaw (rad)', color='purple')
axs[1].set_xlabel('Time (s)')
axs[1].set_ylabel('Yaw (rad)')
axs[1].set_title('Yaw over Time')
axs[1].legend()
axs[1].grid(True)

fig1.tight_layout()

# ========================
# 图 2: 线速度 vx, vy 关于时间
# ========================
fig2, ax2 = plt.subplots(figsize=(10, 4))
ax2.plot(time, vx, label='Linear Velocity X')
ax2.plot(time, vy, label='Linear Velocity Y')
ax2.set_xlabel('Time (s)')
ax2.set_ylabel('Velocity (m/s)')
ax2.set_title('Linear Velocities over Time')
ax2.legend()
ax2.grid(True)
fig2.tight_layout()

# ========================
# 图 3: XY 轨迹图 + 路标线
# ========================
waypoints = [
    [-7.0,  0.00], [-5.5,  0.00], [-4.4,  0.05], [-3.3,  0.05], [-2.2,  0.03], [-1.1, -0.01], [0.0, -0.01],
    [1.1,  0.02], [2.2,  0.05], [3.3,  0.01], [4.4,  0.00], [5.4,  0.00], [6.4,  0.00],  # row 0
    [6.4,  1.00], [5.4,  1.00], [4.3,  1.00], [3.2,  1.00], [2.1,  0.97], [1.0,  0.97], [-0.1,  0.95], [-1.2,  0.95],
    [-2.3,  0.95], [-3.4,  1.00], [-4.5,  1.00], [-5.5,  1.00], [-6.5,  1.00],  # row 1
    [-6.5,  2.01], [-5.5,  2.01], [-4.4,  2.00], [-3.3,  2.00], [-2.2,  2.00], [-1.1,  1.99], [0.0,  1.99],
    [1.1,  2.02], [2.2,  2.01], [3.3,  2.01], [4.4,  2.00], [5.4,  2.00], [6.4,  2.00],  # row 2
]
waypoints = np.array(waypoints)

fig3, ax3 = plt.subplots(figsize=(10, 6))
ax3.plot(x, y, 'b-', label='Robot Trajectory')
ax3.plot(waypoints[:, 0], waypoints[:, 1], 'r--o', label='Planned Waypoints')
ax3.set_xlabel('X (m)')
ax3.set_ylabel('Y (m)')
ax3.set_title('Robot XY Trajectory and Planned Path')
ax3.axis('equal')  # 保持比例尺一致
ax3.grid(True)
ax3.legend()
fig3.tight_layout()

# 显示所有图
plt.show()
