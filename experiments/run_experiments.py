# -*- coding: utf-8 -*-
"""
run_experiments.py —— TSP-D 批量实验脚本

功能：
1. 自动生成默认测试数据（n=8/10/12 × uniform/1-center/2-center）。
2. 自动编译 src/dp-tspd.cpp、src/rdp-tspd.cpp，可选编译 astar-tspd.cpp。
3. 批量运行 DP / RDP / A*。
4. 汇总 experiments/results.csv。
5. 如果安装了 matplotlib，自动生成 experiments/plots 下的对比图。

常用命令：
    python experiments/run_experiments.py
    python experiments/run_experiments.py --algos dp rdp astar
    python experiments/run_experiments.py --Ls 1 2 3 4 5
    python experiments/run_experiments.py --include-fsu

说明：
- DP 是精确算法，L 记为 NA。
- RDP 是限制型 DP，L 越大越接近精确 DP，但通常越慢。
- 默认不跑 FSU n=15 数据，避免慢；需要时加 --include-fsu。
"""

from __future__ import annotations

import argparse
import csv
import os
import re
import shutil
import subprocess
import sys
from pathlib import Path
from typing import Dict, List, Optional

ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / "src"
DATA = ROOT / "data"
EXP = ROOT / "experiments"
PLOTS = EXP / "plots"

SOURCE_FILES = {
    "dp": SRC / "dp-tspd.cpp",
    "rdp": SRC / "rdp-tspd.cpp",
    "astar": SRC / "astar-tspd.cpp",
}


def exe_path(algo: str) -> Path:
    """返回可执行文件路径。Windows 下使用 .exe，Linux/macOS 下也可运行带 .exe 后缀的文件。"""
    return SRC / f"{algo}-tspd.exe"


def run_cmd(cmd: List[str], cwd: Path = ROOT, timeout: Optional[int] = None) -> subprocess.CompletedProcess:
    print("[cmd]", " ".join(map(str, cmd)))
    return subprocess.run(
        cmd,
        cwd=str(cwd),
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        timeout=timeout,
    )


def ensure_data() -> None:
    """如果默认 9 个实例不存在，就调用 data/gen_instances.py 生成。"""
    expected = [
        DATA / f"tspd-{kind}-{idx:02d}.txt"
        for idx, kind in enumerate(
            ["uniform", "uniform", "uniform", "1-center", "1-center", "1-center", "2-center", "2-center", "2-center"],
            start=1,
        )
    ]
    if all(p.exists() for p in expected):
        print("[data] 默认实例已存在，跳过生成。")
        return
    gen = DATA / "gen_instances.py"
    if not gen.exists():
        raise FileNotFoundError("找不到 data/gen_instances.py，无法自动生成实例。")
    print("[data] 正在生成默认 9 个实例...")
    cp = run_cmd([sys.executable, str(gen)], cwd=ROOT)
    if cp.returncode != 0:
        print(cp.stdout)
        print(cp.stderr)
        raise RuntimeError("生成实例失败。")


def compile_algorithms(algos: List[str], cxx: str, no_compile: bool = False) -> None:
    """编译所需算法。"""
    if no_compile:
        print("[build] 已指定 --no-compile，跳过编译。")
        return
    if shutil.which(cxx) is None:
        raise RuntimeError(f"找不到编译器 {cxx}。请先安装 g++ / MinGW，或使用 --no-compile。")

    flags = ["-std=c++17", "-O2", "-Wall", "-Wextra"]
    for algo in algos:
        src = SOURCE_FILES[algo]
        out = exe_path(algo)
        if not src.exists():
            raise FileNotFoundError(f"找不到源码：{src}")
        print(f"[build] 编译 {src.name} -> {out.name}")
        cp = run_cmd([cxx, *flags, str(src), "-o", str(out)], cwd=ROOT)
        if cp.returncode != 0:
            print(cp.stdout)
            print(cp.stderr)
            raise RuntimeError(f"编译 {algo} 失败。")


def collect_instances(include_fsu: bool) -> List[Path]:
    """收集要跑的数据集。默认只跑自动生成的 tspd-* 数据。"""
    instances = sorted(DATA.glob("tspd-*.txt"))
    if not include_fsu:
        instances = [p for p in instances if not p.name.startswith("tspd-fsu")]
    if not instances:
        raise RuntimeError("没有找到可运行的数据文件：data/tspd-*.txt")
    return instances


def parse_summary(result_file: Path) -> Dict[str, str]:
    """解析结果文件 [summary] 那一行。"""
    text = result_file.read_text(encoding="utf-8", errors="ignore")
    summary_line = ""
    for line in text.splitlines():
        if line.startswith("instance="):
            summary_line = line.strip()
            break
    if not summary_line:
        raise RuntimeError(f"结果文件中没有 summary 行：{result_file}")

    # 格式：instance=xxx  algo=dp  n=8  L=NA  obj=123  time_ms=4  status=OPTIMAL
    pairs = dict(re.findall(r"(\w+)=([^\s]+)", summary_line))
    pairs["result_file"] = str(result_file.relative_to(ROOT))
    return pairs


def run_one(algo: str, instance: Path, L: Optional[int], timeout: int) -> Dict[str, str]:
    """运行一次实验，并返回解析后的 summary。"""
    exe = exe_path(algo)
    if not exe.exists():
        raise FileNotFoundError(f"找不到可执行文件：{exe}")

    stem = instance.stem
    if L is not None:
        # rdp 或 astar 带 L 参数
        out = EXP / f"{algo}-{stem}-L{L}.txt"
        cmd = [str(exe), "--input", str(instance), "--output", str(out), "--L", str(L)]
    else:
        out = EXP / f"{algo}-{stem}.txt"
        cmd = [str(exe), "--input", str(instance), "--output", str(out)]

    try:
        cp = run_cmd(cmd, cwd=ROOT, timeout=timeout)
        if cp.returncode != 0:
            print(cp.stdout)
            print(cp.stderr)
            return {
                "instance": instance.stem,
                "algo": algo,
                "n": "NA",
                "L": str(L) if L is not None else "NA",
                "obj": "NA",
                "time_ms": "NA",
                "status": "ERROR",
                "result_file": str(out.relative_to(ROOT)),
                "message": cp.stderr.strip().replace("\n", " | ")[:300],
            }
        row = parse_summary(out)
        row["message"] = ""
        return row
    except subprocess.TimeoutExpired:
        return {
            "instance": instance.stem,
            "algo": algo,
            "n": "NA",
            "L": str(L) if L is not None else "NA",
            "obj": "NA",
            "time_ms": "NA",
            "status": "TIMEOUT",
            "result_file": str(out.relative_to(ROOT)),
            "message": f"超过 {timeout} 秒未完成",
        }


def write_csv(rows: List[Dict[str, str]]) -> Path:
    """写 results.csv。"""
    out_csv = EXP / "results.csv"
    fields = ["instance", "algo", "n", "L", "obj", "time_ms", "status", "result_file", "message"]
    with out_csv.open("w", encoding="utf-8-sig", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        for r in rows:
            writer.writerow({k: r.get(k, "") for k in fields})
    print(f"[csv] 已生成 {out_csv.relative_to(ROOT)}")
    return out_csv


def safe_float(x: str) -> Optional[float]:
    try:
        return float(x)
    except Exception:
        return None


def make_plots(rows: List[Dict[str, str]]) -> None:
    """生成简单图表；没有 matplotlib 时自动跳过。"""
    try:
        import matplotlib.pyplot as plt  # type: ignore
    except Exception:
        print("[plots] 未安装 matplotlib，跳过画图。")
        return

    ok = [r for r in rows if r.get("status") in ("OPTIMAL", "FEASIBLE")]
    if not ok:
        print("[plots] 没有成功结果，跳过画图。")
        return
    PLOTS.mkdir(exist_ok=True)

    # 图 1：不同 n 下各算法平均运行时间
    groups: Dict[tuple, List[float]] = {}
    for r in ok:
        n = r.get("n", "NA")
        algo = r.get("algo", "") if r.get("algo") != "rdp" else f"rdp-L{r.get('L')}"
        t = safe_float(r.get("time_ms", ""))
        if n != "NA" and t is not None:
            groups.setdefault((int(n), algo), []).append(t)
    if groups:
        ns = sorted({k[0] for k in groups})
        algos = sorted({k[1] for k in groups})
        for algo in algos:
            ys = []
            xs = []
            for n in ns:
                vals = groups.get((n, algo), [])
                if vals:
                    xs.append(n)
                    ys.append(sum(vals) / len(vals))
            if xs:
                plt.plot(xs, ys, marker="o", label=algo)
        plt.xlabel("n (number of locations, including depot)")
        plt.ylabel("Average running time / ms")
        plt.title("Running time comparison")
        plt.legend()
        plt.tight_layout()
        plt.savefig(PLOTS / "time_by_n.png", dpi=200)
        plt.close()
        print(f"[plots] 已生成 {PLOTS.relative_to(ROOT) / 'time_by_n.png'}")

    # 图 2：RDP 相对 DP 的平均误差率 gap = (rdp - dp) / dp
    dp_obj: Dict[str, float] = {}
    for r in ok:
        if r.get("algo") == "dp":
            v = safe_float(r.get("obj", ""))
            if v is not None and v > 0:
                dp_obj[r["instance"]] = v
    gap_by_L: Dict[int, List[float]] = {}
    for r in ok:
        if r.get("algo") == "rdp" and r.get("instance") in dp_obj:
            obj = safe_float(r.get("obj", ""))
            try:
                L = int(r.get("L", ""))
            except Exception:
                continue
            if obj is not None:
                gap_by_L.setdefault(L, []).append((obj - dp_obj[r["instance"]]) / dp_obj[r["instance"]] * 100.0)
    if gap_by_L:
        xs = sorted(gap_by_L)
        ys = [sum(gap_by_L[L]) / len(gap_by_L[L]) for L in xs]
        plt.plot(xs, ys, marker="o")
        plt.xlabel("Restricted parameter L")
        plt.ylabel("Average objective gap / %")
        plt.title("Solution quality of restricted DP")
        plt.tight_layout()
        plt.savefig(PLOTS / "gap_by_L.png", dpi=200)
        plt.close()
        print(f"[plots] 已生成 {PLOTS.relative_to(ROOT) / 'gap_by_L.png'}")


def main() -> None:
    parser = argparse.ArgumentParser(description="TSP-D 批量实验脚本")
    parser.add_argument("--algos", nargs="+", default=["dp", "rdp"], choices=["dp", "rdp", "astar"], help="要运行的算法")
    parser.add_argument("--Ls", nargs="+", type=int, default=[1, 2, 3, 4], help="RDP 的 L 参数列表")
    parser.add_argument("--include-fsu", action="store_true", help="是否额外运行 data/tspd-fsu.p01-10.txt")
    parser.add_argument("--timeout", type=int, default=120, help="单次实验超时时间，单位秒")
    parser.add_argument("--cxx", default="g++", help="C++ 编译器，默认 g++")
    parser.add_argument("--no-compile", action="store_true", help="跳过编译，直接运行已有 exe")
    args = parser.parse_args()

    EXP.mkdir(exist_ok=True)
    ensure_data()
    compile_algorithms(args.algos, cxx=args.cxx, no_compile=args.no_compile)
    instances = collect_instances(args.include_fsu)

    print("[run] 数据集：")
    for p in instances:
        print("     ", p.relative_to(ROOT))

    rows: List[Dict[str, str]] = []
    for inst in instances:
        if "dp" in args.algos:
            rows.append(run_one("dp", inst, L=None, timeout=args.timeout))
        if "rdp" in args.algos:
            for L in args.Ls:
                rows.append(run_one("rdp", inst, L=L, timeout=args.timeout))
        if "astar" in args.algos:
            rows.append(run_one("astar", inst, L=None, timeout=args.timeout))
            for L in args.Ls:
                rows.append(run_one("astar", inst, L=L, timeout=args.timeout))

    write_csv(rows)
    make_plots(rows)

    print("\n完成。重点查看：")
    print("  experiments/results.csv")
    print("  experiments/plots/time_by_n.png")
    print("  experiments/plots/gap_by_L.png")


if __name__ == "__main__":
    main()
