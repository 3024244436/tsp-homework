// =============================================================================
//  dp-tspd.cpp  ——  TSP-D 精确动态规划（论文 §3.2 / §3.3 / §3.4.1）
//
//  三遍 DP：
//    Pass 1   D_T(S, v, w)   公式 (2)：卡车从 v 经 S 终于 w 的最短路径
//    Pass 2   D_OP(S, v, w)  公式 (3)：起 v 终 w、覆盖 S 的最高效操作
//    Pass 3   D(S, w)        公式 (4)：自 depot 起、覆盖 S、终于 w 的最优序列
//
//  复杂度：第三遍 O(3^n · n^2)（定理 3.2）
//
//  适用规模：n ≤ ~14 单机可在分钟级跑完（取决于硬件）
//
//  调用：
//    ./dp-tspd --input <data.txt> --output <result.txt>
// =============================================================================
#include "header.h"

int main(int argc, char** argv) {
    CliArgs args;
    if (!parse_args(argc, argv, args)) return 1;

    Instance inst;
    if (!load_instance(args.input, inst)) return 1;
    const int n = inst.n;
    if (n > 18) {
        std::fprintf(stderr, "[warn] n=%d 对精确 DP 过大，可能内存/超时\n", n);
    }
    const int N = 1 << n;                       // 2^n 个子集
    const int depot = 0;

    Timer timer;

    // ============== Pass 1：D_T(S, v, w)  ——  公式 (2) ==========================
    //   存储为 dt[mask * n * n + v * n + w]
    //   mask 为 S 的位掩码，要求 w ∈ S 且 v ∉ S（外层保证），否则不填
    vector<double> dt(static_cast<size_t>(N) * n * n, INF);
    auto DT = [&](int mask, int v, int w) -> double& {
        return dt[(static_cast<size_t>(mask) * n + v) * n + w];
    };

    // 基础：|S|=1，S={w}
    for (int w = 0; w < n; ++w) {
        int mask = 1 << w;
        for (int v = 0; v < n; ++v) if (v != w)
            DT(mask, v, w) = inst.c(v, w);
    }
    // 递推：按 |S| 递增
    for (int mask = 1; mask < N; ++mask) {
        if (__builtin_popcount(mask) < 2) continue;
        for (int w = 0; w < n; ++w) {
            if (!(mask & (1 << w))) continue;
            int sub = mask ^ (1 << w);          // S \ {w}
            for (int v = 0; v < n; ++v) {
                if (mask & (1 << v)) continue;  // v ∉ S
                double best = INF;
                for (int u = 0; u < n; ++u) {
                    if (!(sub & (1 << u))) continue;
                    double val = DT(sub, v, u);
                    if (val + inst.c(u, w) < best) best = val + inst.c(u, w);
                }
                DT(mask, v, w) = best;
            }
        }
    }

    // ============== Pass 2：D_OP(S, v, w)  ——  公式 (3) =========================
    //   同样按 (mask, v, w) 三维存储；记录最优 drone 节点 d（用于解恢复）
    vector<double> dop(static_cast<size_t>(N) * n * n, INF);
    vector<int>    dop_d(static_cast<size_t>(N) * n * n, -1);   // -1 表示无 drone
    auto DOP = [&](int mask, int v, int w) -> double& {
        return dop[(static_cast<size_t>(mask) * n + v) * n + w];
    };
    auto DOP_D = [&](int mask, int v, int w) -> int& {
        return dop_d[(static_cast<size_t>(mask) * n + v) * n + w];
    };

    for (int mask = 1; mask < N; ++mask) {
        for (int w = 0; w < n; ++w) {
            if (!(mask & (1 << w))) continue;
            for (int v = 0; v < n; ++v) {
                if (mask & (1 << v)) continue;
                if (mask == (1 << w)) {                 // 基础：S={w}
                    DOP(mask, v, w) = DT(mask, v, w);   // = c(v,w)
                    DOP_D(mask, v, w) = -1;
                    continue;
                }
                double best = INF;
                int    best_d = -1;
                for (int d = 0; d < n; ++d) {
                    if (d == w || d == v) continue;
                    if (!(mask & (1 << d))) continue;
                    int sub = mask ^ (1 << d);          // S \ {d}
                    double drone_time = inst.cD(v, d) + inst.cD(d, w);
                    if (drone_time > inst.drone_endurance) continue;
                    double truck_time = DT(sub, v, w);  // 卡车走 S\{d}
                    if (truck_time >= INF) continue;
                    double op_t = std::max(drone_time, truck_time);
                    if (op_t < best) { best = op_t; best_d = d; }
                }
                DOP(mask, v, w) = best;
                DOP_D(mask, v, w) = best_d;
            }
        }
    }

    // ============== Pass 3：D(S, w)  ——  公式 (4) =============================
    //   每个状态记录前驱 (S', u) 与本次 operation 的覆盖集 T，用于解恢复
    vector<double> D (static_cast<size_t>(N) * n, INF);
    vector<int>    pre_u(static_cast<size_t>(N) * n, -1);
    vector<int>    pre_T(static_cast<size_t>(N) * n, 0);    // operation 覆盖的位集
    auto Dx     = [&](int mask, int w) -> double& { return D    [(size_t)mask * n + w]; };
    auto PreU   = [&](int mask, int w) -> int&    { return pre_u[(size_t)mask * n + w]; };
    auto PreT   = [&](int mask, int w) -> int&    { return pre_T[(size_t)mask * n + w]; };

    // 源状态 D({depot}, depot) = 0
    Dx(1 << depot, depot) = 0;

    // 按 |S| 递增枚举
    vector<int> masks_by_pc[32];
    for (int m = 1; m < N; ++m) masks_by_pc[__builtin_popcount(m)].push_back(m);

    // Backward DP：D(S, w) = min_{T⊆S, w∈T, u∈S_pre, u∉T} D(S_pre, u) + D_OP(T, u, w)
    //   其中 S_pre = (S \ T) ∪ {depot}
    //   覆盖两类操作：
    //     非末次 op：w 是客户（非 depot），T 不含 depot，S_pre 自动含 depot
    //     末次 op：  w = depot，T 含 depot，需手动把 depot 并回 S_pre
    for (int pc = 2; pc <= n; ++pc) {
        for (int S : masks_by_pc[pc]) {
            if (!(S & (1 << depot))) continue;          // 序列必含 depot
	      std::vector<int> ws;
	      for (int w = 0; w < n; ++w) if (S & (1 << w)) ws.push_back(w);
	      if (S == ((1 << n) - 1)) { auto it = std::find(ws.begin(), ws.end(), depot); if (it != ws.end()) { ws.erase(it); ws.push_back(depot); } }
	      for (int w : ws) {
                if (!(S & (1 << w))) continue;
                if (w == depot && S != ((1 << n) - 1)) continue; // 回 depot 仅终态
                // T 须含 w；枚举 Tp = T \ {w} ⊆ S \ {w}
                int T_base = S ^ (1 << w);
                for (int Tp = T_base; ; Tp = (Tp - 1) & T_base) {
                    int T = Tp | (1 << w);
                    int S_pre = (S & ~T) | (1 << depot);    // 保证 depot 在前驱中
                    int u_cand = S_pre & ~T;                // u ∈ S_pre, u ∉ T
                    while (u_cand) {
                        int u_bit = u_cand & -u_cand;
                        int u = __builtin_ctz(u_bit);
                        u_cand ^= u_bit;
                        if (u == w) continue;
                        double prev = Dx(S_pre, u);
                        if (prev >= INF) continue;
                        double op = DOP(T, u, w);
                        if (op >= INF) continue;
                        if (prev + op < Dx(S, w)) {
                            Dx(S, w) = prev + op;
                            PreU(S, w) = u;
                            PreT(S, w) = T;
                        }
                    }
                    if (Tp == 0) break;
                }
            }
        }
    }

    // 终态：覆盖全集 V，回到 depot
    int FULL = (1 << n) - 1;
    Solution sol;
    sol.obj = Dx(FULL, depot);
    sol.status = (sol.obj >= INF) ? "INFEASIBLE" : "OPTIMAL";

    // -------------- 解恢复：回溯 PreU / PreT 重建路径 ------------------------
    if (sol.status == "OPTIMAL") {
        // 沿 PreU / PreT 回溯，收集 operation 序列（终态 → 源）
        vector<std::tuple<int,int,int>> ops_rev;    // (u, w, T)
        int cur_S = FULL, cur_w = depot;
        while (cur_S != (1 << depot)) {
            int u = PreU(cur_S, cur_w);
            int T = PreT(cur_S, cur_w);
            if (u < 0) break;
            ops_rev.emplace_back(u, cur_w, T);
            cur_S = (cur_S & ~T) | (1 << depot);    // S_pre = (S \ T) ∪ {depot}
            cur_w = u;
        }
        std::reverse(ops_rev.begin(), ops_rev.end());

        // 由 operation 序列构造卡车 route + drone sorties
        sol.truck_route.push_back(depot);
        for (auto& [u, w, T] : ops_rev) {
            int d = DOP_D(T, u, w);             // 该 operation 的 drone 节点
            if (d >= 0) {
                // 回溯 D_T(T\{d}, u, w)，展开卡车在该 operation 中经过的全部节点
                // 路径形态：u → t1 → t2 → ... → tk → w（t* 为 truck-only 节点）
                int truck_set = T ^ (1 << d);
                vector<int> tail;
                int Smask = truck_set, end = w;
                while (true) {
                    tail.push_back(end);                // 先 push 当前节点
                    int sub2 = Smask ^ (1 << end);
                    if (sub2 == 0) break;               // 已到 u 的紧后继
                    int bestU = -1; double bestV = INF;
                    for (int uu = 0; uu < n; ++uu) {
                        if (!(sub2 & (1 << uu))) continue;
                        double v = DT(sub2, u, uu) + inst.c(uu, end);
                        if (v < bestV) { bestV = v; bestU = uu; }
                    }
                    if (bestU < 0) break;
                    end = bestU;
                    Smask = sub2;
                }
                std::reverse(tail.begin(), tail.end()); // 反转为 u→...→w 的顺序
                for (int x : tail) sol.truck_route.push_back(x);
                sol.sorties.push_back({u, d, w});
            } else {
                sol.truck_route.push_back(w);
            }
        }
    }

    long long elapsed = timer.ms();
    write_result(args.output, inst, "dp", -1, sol, elapsed);
    std::printf("dp-tspd  instance=%s  n=%d  obj=%.4f  time_ms=%lld\n",
                inst.name.c_str(), n, sol.obj, elapsed);
    return 0;
}
