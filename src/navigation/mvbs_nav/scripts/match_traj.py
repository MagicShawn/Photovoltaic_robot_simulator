import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import os

# ========= 用户自定义输入输出 =========
input_files = [
    'data/waypoints/real_test/trajectory_data_2_1.csv',
    'data/waypoints/real_test/trajectory_data_2_2.csv',
    'data/waypoints/real_test/trajectory_data_2_3.csv'
]

output_files = [
    'data/waypoints/real_test/aligned_traj_2_1.csv',
    'data/waypoints/real_test/aligned_traj_2_2.csv',
    'data/waypoints/real_test/aligned_traj_2_3.csv'
]

# ========= 创建输出目录 =========
os.makedirs(os.path.dirname(output_files[0]), exist_ok=True)

# ========= 核心函数 =========
def rigid_transform_2D(A, B):
    assert A.shape == B.shape
    centroid_A = np.mean(A, axis=0)
    centroid_B = np.mean(B, axis=0)
    AA = A - centroid_A
    BB = B - centroid_B
    H = AA.T @ BB
    U, _, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T
    if np.linalg.det(R) < 0:
        Vt[1, :] *= -1
        R = Vt.T @ U.T
    t = centroid_B.T - R @ centroid_A.T
    return R, t

def apply_transform(P, R, t):
    return (R @ P.T).T + t.T

# ========= 读取并对齐轨迹 =========
original_dfs = [pd.read_csv(f) for f in input_files]
aligned_dfs = [original_dfs[0]]
base_coords = original_dfs[0][['position_x', 'position_y']].values

for i in range(1, len(original_dfs)):
    df = original_dfs[i]
    coords = df[['position_x', 'position_y']].values
    min_len = min(len(base_coords), len(coords))
    R, t = rigid_transform_2D(coords[:min_len], base_coords[:min_len])
    aligned = apply_transform(coords, R, t)

    df_aligned = df.copy()
    df_aligned['position_x'] = aligned[:, 0]
    df_aligned['position_y'] = aligned[:, 1]
    aligned_dfs.append(df_aligned)

    angle_deg = np.degrees(np.arctan2(R[1, 0], R[0, 0]))
    print(f"轨迹 {i+1} 对齐完成：旋转 {angle_deg:.2f}°，平移 ({t[0]:.4f}, {t[1]:.4f})")

# ========= 输出对齐后的 CSV =========
for df, path in zip(aligned_dfs, output_files):
    df.to_csv(path, index=False)
    print(f"写入文件: {path}")

# ========= 绘制轨迹对齐图 =========
plt.figure(figsize=(10, 6))
for i in range(len(original_dfs)):
    orig = original_dfs[i]
    aligned = aligned_dfs[i]
    plt.plot(orig['position_x'].values, orig['position_y'].values, '--', label=f'ori traj {i+1}')
    plt.plot(aligned['position_x'].values, aligned['position_y'].values, label=f'aligned traj {i+1}')
plt.xlabel('X')
plt.ylabel('Y')
plt.title('Trajectory Alignment')
plt.axis('equal')
plt.grid(True)
plt.legend()
plt.tight_layout()
plt.show()
