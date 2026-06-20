# =============================================================================
# fsu_convert.py  ——  把 FSU P01 数据集转成 TSP-D 实例
#
#   输入：data/tsp-FSU.p01-07.txt   (15 个 xy 坐标，第一行约 (0,0))
#   输出：data/tspd-fsu.p01-10.txt  (符合我们项目格式，加无人机参数)
#
#   论文 §4 默认参数：α = drone_speed / truck_speed = 2，无人机续航不限。
# =============================================================================
import os

HERE = os.path.dirname(os.path.abspath(__file__))
SRC  = os.path.join(HERE, 'tsp-FSU.p01-07.txt')
DST  = os.path.join(HERE, 'tspd-fsu.p01-10.txt')

coords = []
with open(SRC, encoding='utf-8') as f:
    for line in f:
        line = line.strip()
        if not line: continue
        parts = line.split()
        if len(parts) >= 2:
            try:
                x, y = float(parts[0]), float(parts[1])
                coords.append((x, y))
            except ValueError:
                pass

n = len(coords)
print(f'读到 {n} 个坐标')

with open(DST, 'w', encoding='utf-8') as f:
    f.write(f'# converted from FSU P01 dataset (15 cities, optimal TSP tour = 291)\n')
    f.write(f'# 第 0 号为 depot（原数据首座城市，坐标约 (0,0)）\n')
    f.write(f'n={n}\n')
    f.write(f'truck_speed=1.0\n')
    f.write(f'drone_speed=2.0\n')
    f.write(f'drone_endurance=1e9\n')
    for i, (x, y) in enumerate(coords):
        f.write(f'{i}  {x:.6f}  {y:.6f}\n')

print(f'wrote {DST}')
