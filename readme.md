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

生成器：
    python data/gen_instances.py                           生成默认 9 个实例
    python data/gen_instances.py --n 14 --kind uniform --seed 42 --out data/tspd-uniform-99.txt

# 结果文件说明

文件名格式：<算法>-<实例名>-<参数>.txt
位置：experiments/
    例：dp-tspd-uniform-01-inf.txt    （精确 DP，参数 L=∞）
        rdp-tspd-uniform-01-2.txt     （限制型 DP，L=2）

格式（两块）：

    [summary]
    instance=...  algo=...  n=...  L=...  obj=...  time_ms=...  status=...

    [solution]
    truck_route: 0 6 4 7 0
    drone_sorties:
        launch=0  customer=3  rendezvous=6
        launch=6  customer=2  rendezvous=4
        ...
    total_cost: 225.18

汇总：
    python experiments/run_experiments.py    # 批量跑、生成 results.csv 与 plots/

# 目录结构

3024244436-4-1/
├── src/
│   ├── header.h
│   ├── dp-tspd.cpp     dp-tspd.exe       精确 DP
│   └── rdp-tspd.cpp    rdp-tspd.exe      限制型 DP
├── data/
│   ├── gen_instances.py
│   └── tspd-<分布>-<序号>.txt × 9        三种分布 × 三种规模
├── experiments/
│   ├── run_experiments.py                批量实验脚本
│   ├── results.csv                       汇总
│   ├── plots/                            画图
│   └── <算法>-<实例>-<参数>.txt          单次结果
├── reports/
│   ├── draft/                            中间稿（翻译、伪代码）
│   ├── 实验报告.docx / report.docx
│   ├── 讲解PPT.pptx / report.ppt
│   └── report.mp4                        5-10 分钟讲解视频
├── readme.md
├── member.txt
└── build.bat                             一键编译

# 批量实验脚本使用说明

本项目已补充 `experiments/run_experiments.py`，用于完成第 3 周实验数据整理工作。

默认运行：

```bash
python experiments/run_experiments.py
```

默认会执行以下流程：

1. 如果 `data/` 中缺少默认实例，则调用 `data/gen_instances.py` 生成 9 个实例；
2. 编译 `dp-tspd.cpp` 和 `rdp-tspd.cpp`；
3. 对每个实例运行精确 DP；
4. 对每个实例运行限制型 DP，默认测试 `L=1,2,3,4`；
5. 汇总结果到 `experiments/results.csv`；
6. 如果安装了 `matplotlib`，自动生成：
   - `experiments/plots/time_by_n.png`
   - `experiments/plots/gap_by_L.png`

如果想把 A* 也加入对比，可以运行：

```bash
python experiments/run_experiments.py --algos dp rdp astar
```

如果想修改限制型 DP 的 L 参数，可以运行：

```bash
python experiments/run_experiments.py --Ls 1 2 3 4 5
```

如果想把 `data/tspd-fsu.p01-10.txt` 也加入实验，可以运行：

```bash
python experiments/run_experiments.py --include-fsu
```

如果已经用 `build.bat` 编译过，不想重新编译，可以运行：

```bash
python experiments/run_experiments.py --no-compile
```
