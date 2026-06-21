# 项目说明：带无人机的旅行商问题（TSP-D）动态规划求解

选题类型：2.4 前沿算法实现类
参考文献：Bouman P, Agatz N, Schmidt M. Dynamic programming approaches for the traveling salesman problem with drone. *Networks*, 2018, 72(4): 528-542.

构建：在项目根目录运行 `build.bat`（需 g++ / MinGW）。

# 代码文件说明

文件名格式：算法名-问题名.{cpp,h}

src/header.h
    功能：公共类型（Instance/Solution）、CSV 风格输入读取、结果文件写出、计时器
    依赖：仅 C++17 标准库

src/dp-tspd.cpp        编译为 src/dp-tspd.exe
    功能：精确动态规划求解 TSP-D —— 实现论文 §3.2 (D_T)、§3.3 (D_OP)、§3.4.1 (D)
    复杂度：O(3^n · n^2)（定理 3.2）；适用 n ≤ 14（更大需更多内存）
    调用：dp-tspd.exe --input <data.txt> --output <result.txt>

src/rdp-tspd.cpp       编译为 src/rdp-tspd.exe
    功能：限制型动态规划 —— 实现论文 §3.5（限制每次 operation 中 truck-only 节点数 ≤ L）
    L 足够大时退化为精确 DP；L 较小时更快但可能次优
    调用：rdp-tspd.exe --input <data.txt> --output <result.txt> --L <int>

src/astar-tspd.cpp      编译为 src/astar-tspd.exe
    功能：A* 启发式搜索 —— 实现论文 §3.4.2，在状态图上利用 MST 下界引导搜索
    保证找到精确最优解，大 n 时分支剪枝可能比 DP 更快
    调用：astar-tspd.exe --input <data.txt> --output <result.txt>

# 数据文件说明

文件名格式：tspd-<分布>-<两位序号>.txt
位置：data/

格式（# 为注释，键值对 + 坐标）：
    n=<int>                    位置数（含 depot）
    truck_speed=<float>
    drone_speed=<float>
    drone_endurance=<float>    无人机最大单程飞行时间（设很大值表示无限制）
    0  x0  y0                  id=0 为 depot
    1  x1  y1                  其他为客户
    ...

## 数据来源

| 来源 | 文件 | 类型 |
|------|------|------|
| TSPLIB 标准库 | burma14.tsp (n=14), ulysses16.tsp (n=16) | 原始 .tsp GEO 坐标 |
| FSU 测试集 | tsp-FSU.p01-*.txt (n=15) | 原始坐标/矩阵 |
| 自动生成 | gen_instances.py → 9 个 tspd-*.txt | 三种分布 × 三种规模 |

## 转换脚本

    python data/tsplib_convert.py burma14.tsp --out tspd-burma14-11.txt  # TSPLIB → TSP-D
    python data/tsplib_convert.py ulysses16.tsp --out tspd-ulysses16-12.txt
    python data/fsu_convert.py                                              # FSU → TSP-D
    python data/gen_instances.py                                            # 生成默认 9 个实例

# 结果文件说明

文件名格式：<算法>-<实例名>-<参数>.txt
位置：experiments/

格式（两块）：

    [summary]
    instance=...  algo=...  n=...  L=...  obj=...  time_ms=...  status=...

    [solution]
    truck_route: 0 6 4 7 0
    drone_sorties:
        launch=0  customer=3  rendezvous=6
        ...
    total_cost: 225.18

汇总：
    python experiments/run_experiments.py    # 批量跑、生成 results.csv 与 plots/

# 实验最新结果（2026-06-21）

共 **72 组实验**：DP / RDP(L=1,2,3,4) / A* × 12 实例（n=8~16）

## 核心发现

| 结论 | 数据 |
|------|------|
| RDP k=1 比 DP 快 | n=16: 8.6× (1.3s vs 11.3s) |
| RDP 解质量 | k=4 最大误差仅 1.2% (ulysses16) |
| FSU n=15 | **k=1 即达最优**（5.7× 加速，零误差） |
| A* vs DP 交叉 | n=16 时 A* (7.0s) 首次比 DP (11.3s) 快 |
| 分布鲁棒性 | 1-center 全部 k=1 即最优 |

详细分析见 `reports/实验结果分析.md`。

# 目录结构

3024244436-4-2/
├── src/
│   ├── header.h
│   ├── dp-tspd.cpp     dp-tspd.exe        精确 DP
│   ├── rdp-tspd.cpp    rdp-tspd.exe      限制型 DP
│   └── astar-tspd.cpp  astar-tspd.exe    A* 搜索
├── data/
│   ├── gen_instances.py / tsplib_convert.py / fsu_convert.py
│   ├── burma14.tsp / ulysses16.tsp        TSPLIB 原始
│   ├── tsp-FSU.p01-*.txt                 FSU 原始
│   └── tspd-*.txt × 12                   TSP-D 实例
├── experiments/
│   ├── run_experiments.py                批量实验脚本
│   ├── results.csv                       72 组汇总
│   ├── plots/time_by_n.png              时间对比图
│   ├── plots/gap_by_L.png               误差率 vs L 图
│   └── <算法>-<实例>-<参数>.txt          单次结果
├── reports/
│   ├── draft/                            论文翻译、参考文献
│   ├── 实验结果分析.md                    完整分析报告
│   ├── 实验报告.docx
│   └── 完整汇报PPT_TSPD动态规划实验.pptx
├── readme.md
└── .gitignore

# 批量实验脚本使用说明

默认运行（DP + RDP）：

```bash
python experiments/run_experiments.py
```

三方完整对比（DP + RDP + A*）：

```bash
python experiments/run_experiments.py --algos dp rdp astar
```

加入 FSU 数据：

```bash
python experiments/run_experiments.py --include-fsu
```

跳过编译（已完成编译时）：

```bash
python experiments/run_experiments.py --no-compile
```

自定义 L 参数：

```bash
python experiments/run_experiments.py --Ls 1 2 3 4 5
```
