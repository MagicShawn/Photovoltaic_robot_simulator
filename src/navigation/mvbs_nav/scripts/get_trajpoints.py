import yaml
import csv

# 输入和输出文件路径
input_file = "data/pt2pt/sim_test/test_3_3_global_traj.txt"  # 你的txt文件名
output_file = "data/pt2pt/sim_test/test_3_3_global_traj.csv"

# 读取txt文件内容
with open(input_file, 'r') as f:
    data = yaml.safe_load(f)

# 提取x和y坐标
positions = []
for pose_entry in data['poses']:
    x = pose_entry['pose']['position']['x']
    y = pose_entry['pose']['position']['y']
    positions.append((x, y))

# 写入CSV文件
with open(output_file, 'w', newline='') as csvfile:
    writer = csv.writer(csvfile)
    writer.writerow(['x', 'y'])  # 写入表头
    writer.writerows(positions)

print(f"提取完成，结果已保存为 {output_file}")
