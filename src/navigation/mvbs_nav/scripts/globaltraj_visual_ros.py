import matplotlib.pyplot as plt
import re

data_path = 'data/pt2pt/kinoastar_traj.txt'

# 读取文件内容
with open(data_path, "r") as file:
    data = file.read()

# 用正则提取所有 position 中的 x 和 y 值
pattern = r"position:\s+x:\s*([-\d\.]+)\s+y:\s*([-\d\.]+)"
matches = re.findall(pattern, data)

# 转换为浮点数列表
x_vals, y_vals = zip(*[(float(x), float(y)) for x, y in matches])

# 绘图
plt.figure(figsize=(8, 6))
# plt.plot(x_vals, y_vals, marker='o', markersize=3, linestyle='-', color='blue', label='Global Path')
plt.plot(x_vals, y_vals, color=(32/255, 74/255, 135/255), linewidth=2.5, label='Global Path')
plt.title("Global Path Visualization")
plt.xlabel("X (meters)")
plt.ylabel("Y (meters)")
plt.axis("equal")
plt.grid(True)
plt.legend()
plt.tight_layout()
plt.show()
