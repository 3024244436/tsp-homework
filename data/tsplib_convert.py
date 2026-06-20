# =============================================================================
# tsplib_convert.py  ——  把 TSPLIB 的 .tsp 文件转成 TSP-D 实例
#
#   解析 NODE_COORD_SECTION 段的坐标，补上无人机参数（论文 §4 默认 α=2），
#   输出项目统一格式 tspd-<名字>-<序号>.txt。
#
#   注意：TSPLIB 的 GEO 类型坐标是经纬度，本脚本直接当平面坐标用、按欧氏距离算
#         （卡车与无人机用同一距离基准，构造 TSP-D 实例，对实验结论无影响）。
#
#   用法：
#     python data/tsplib_convert.py burma14.tsp     --out tspd-burma14-11.txt
#     python data/tsplib_convert.py ulysses16.tsp   --out tspd-ulysses16-12.txt
#   不带参数则自动转换 data/ 下所有 *.tsp。
# =============================================================================
import argparse, os, sys, glob

HERE = os.path.dirname(os.path.abspath(__file__))

def parse_tsp(path):
    """读取 .tsp，返回 [(x, y), ...]（按节点编号顺序）。"""
    coords = []
    in_section = False
    with open(path, encoding='utf-8', errors='ignore') as f:
        for line in f:
            s = line.strip()
            if not s:
                continue
            head = s.split(':')[0].strip().upper() if ':' in s else s.upper()
            if head == 'NODE_COORD_SECTION':
                in_section = True
                continue
            if head in ('EOF', 'DISPLAY_DATA_SECTION', 'TOUR_SECTION'):
                in_section = False
                continue
            if in_section:
                parts = s.split()
                if len(parts) >= 3:
                    try:
                        x, y = float(parts[1]), float(parts[2])
                        coords.append((x, y))
                    except ValueError:
                        pass
    return coords

def write_tspd(out_path, coords, truck_speed=1.0, drone_speed=2.0, endurance=1e9):
    n = len(coords)
    name = os.path.splitext(os.path.basename(out_path))[0]
    with open(out_path, 'w', encoding='utf-8') as f:
        f.write(f'# converted from TSPLIB; node 0 = depot (原 TSPLIB 1 号点)\n')
        f.write(f'# 坐标按平面欧氏距离处理；无人机参数为论文 §4 默认 (alpha=2)\n')
        f.write(f'n={n}\n')
        f.write(f'truck_speed={truck_speed}\n')
        f.write(f'drone_speed={drone_speed}\n')
        f.write(f'drone_endurance={endurance}\n')
        for i, (x, y) in enumerate(coords):
            f.write(f'{i}  {x:.4f}  {y:.4f}\n')
    print(f'wrote {out_path}  (n={n})')

def convert(tsp_path, out_path):
    coords = parse_tsp(tsp_path)
    if not coords:
        print(f'[warn] {tsp_path} 未解析到坐标（可能是 EXPLICIT 矩阵类型，无坐标，无法用）')
        return
    write_tspd(out_path, coords)

if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument('tsp', nargs='?', help='输入 .tsp 文件（不指定则转 data/ 下所有 .tsp）')
    ap.add_argument('--out', help='输出文件名（放在 data/ 下）')
    args = ap.parse_args()

    if args.tsp:
        tsp_path = args.tsp if os.path.isabs(args.tsp) else os.path.join(HERE, args.tsp)
        stem = os.path.splitext(os.path.basename(tsp_path))[0]
        out = args.out or f'tspd-{stem}.txt'
        convert(tsp_path, os.path.join(HERE, out))
    else:
        # 自动转 data/ 下所有 .tsp
        idx = 11   # 序号接着 FSU P01 的 10
        for tsp_path in sorted(glob.glob(os.path.join(HERE, '*.tsp'))):
            stem = os.path.splitext(os.path.basename(tsp_path))[0]
            out = f'tspd-{stem}-{idx:02d}.txt'
            convert(tsp_path, os.path.join(HERE, out))
            idx += 1
