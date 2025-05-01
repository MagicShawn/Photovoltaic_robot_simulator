import matplotlib.pyplot as plt
import csv

csv_file = 'data/waypoints/global_traj.csv'

# 读取CSV文件
x = []
y = []
with open(csv_file, 'r') as csvfile:
    reader = csv.reader(csvfile)
    next(reader)  # 跳过标题行
    for row_num, row in enumerate(reader, 2):  # 从第2行开始计数(跳过标题)
        try:
            # 跳过空行或格式不正确的行
            if len(row) < 2 or not row[0].strip() or not row[1].strip():
                print(f"警告: 跳过第 {row_num} 行 - 数据不完整")
                continue
                
            x_val = float(row[0].strip())
            y_val = float(row[1].strip())
            x.append(x_val)
            y.append(y_val)
        except ValueError as e:
            print(f"错误: 第 {row_num} 行数据格式不正确 - {e}")
            continue

# 检查是否有有效数据
if not x:
    print("错误: 没有有效数据可绘制")
    exit()

# 创建图形
plt.figure()
plt.plot(x, y, color=(32/255, 74/255, 135/255), linewidth=2.5, label='Global Path')
plt.title('X-Y Trajectory Plot with Coordinates', fontsize=14)
plt.xlabel('Position X', fontsize=12)
plt.ylabel('Position Y', fontsize=12)
plt.grid(True, linestyle='--', alpha=0.6)

# 在每个点旁边添加坐标标签
for i, (xi, yi) in enumerate(zip(x, y)):
    plt.text(xi, yi, f'({xi:.2f}, {yi:.2f})', 
             fontsize=9, 
             ha='center', 
             va='bottom', 
             bbox=dict(facecolor='white', alpha=0.7, edgecolor='none', boxstyle='round,pad=0.2'))

# 标记起点和终点
if len(x) > 0:
    plt.scatter(x[0], y[0], color='green', s=100, label='Start', zorder=5)
    plt.scatter(x[-1], y[-1], color='red', s=100, label='End', zorder=5)

# 添加图例
plt.legend(fontsize=10)

# 自动调整坐标轴范围以显示所有标签
plt.tight_layout()
plt.show()