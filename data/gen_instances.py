# =============================================================================
# gen_instances.py  ——  按论文 §4 的方式生成 TSP-D 测试实例
#
#   三种分布：uniform / 1-center / 2-center
#   产出 9 个文件：tspd-<分布>-<两位序号>.txt
#
#   用法：
#     python gen_instances.py            # 生成默认 9 个（n=8,10,12 × 3 分布）
#     python gen_instances.py --n 14 --kind uniform --seed 42 --out tspd-uniform-99.txt
# =============================================================================
import argparse, os, random

def gen_uniform(n, rng):
    pts = [(0.0, 0.0)]
    for _ in range(n - 1):
        pts.append((rng.uniform(0, 100), rng.uniform(0, 100)))
    return pts

def gen_kcenter(n, k, rng):
    centers = [(rng.uniform(20, 80), rng.uniform(20, 80)) for _ in range(k)]
    pts = [(0.0, 0.0)]
    for _ in range(n - 1):
        cx, cy = centers[rng.randrange(k)]
        pts.append((cx + rng.gauss(0, 8), cy + rng.gauss(0, 8)))
    return pts

def write_instance(path, n, pts, truck_speed=1.0, drone_speed=2.0, endurance=1e9):
    with open(path, 'w', encoding='utf-8') as f:
        f.write(f'# auto-generated TSP-D instance\n')
        f.write(f'n={n}\n')
        f.write(f'truck_speed={truck_speed}\n')
        f.write(f'drone_speed={drone_speed}\n')
        f.write(f'drone_endurance={endurance}\n')
        for i, (x, y) in enumerate(pts):
            f.write(f'{i}  {x:.4f}  {y:.4f}\n')

def make_one(n, kind, seed, out):
    rng = random.Random(seed)
    if kind == 'uniform':    pts = gen_uniform(n, rng)
    elif kind == '1-center': pts = gen_kcenter(n, 1, rng)
    elif kind == '2-center': pts = gen_kcenter(n, 2, rng)
    else: raise ValueError(kind)
    write_instance(out, n, pts)
    print('wrote', out)

if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument('--n', type=int)
    ap.add_argument('--kind', choices=['uniform', '1-center', '2-center'])
    ap.add_argument('--seed', type=int, default=0)
    ap.add_argument('--out')
    args = ap.parse_args()
    here = os.path.dirname(os.path.abspath(__file__))
    if args.out:
        make_one(args.n, args.kind, args.seed, args.out)
    else:
        idx = 1
        for kind in ['uniform', '1-center', '2-center']:
            for n in [8, 10, 12]:
                fname = f'tspd-{kind}-{idx:02d}.txt'
                make_one(n, kind, 1000 + idx, os.path.join(here, fname))
                idx += 1
