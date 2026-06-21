// =============================================================================
//  rdp-tspd.cpp  ——  TSP-D 限制型动态规划（论文 §3.5）
//
//  与精确 DP 的唯一区别：每次 operation 中 truck-only 节点数 ≤ L
//      truck-only 节点数 = |T| - 2 - (drone_node 是否存在 ? 1 : 0)
//      其中 T 是该 operation 的覆盖集，2 = {起点, 终点}
//
//  实现方式：在 Pass 1 / Pass 2 / Pass 3 中均跳过 |T| > L+2 的 T。
//  当 L 足够大时（L ≥ n-2），退化为精确 DP。
//
//  调用：
//    ./rdp-tspd --input <data.txt> --output <result.txt> --L <int>
// =============================================================================
#include "header.h"

int main(int argc, char** argv) {
    CliArgs args;
    if (!parse_args(argc, argv, args)) return 1;
    if (args.L < 0) {
        std::fprintf(stderr, "rdp-tspd 需要 --L <int>，L≥0\n");
        return 1;
    }

    Instance inst;
    if (!load_instance(args.input, inst)) return 1;
    const int n = inst.n;
    const int N = 1 << n;
    const int depot = 0;
    const int L = args.L;
    const int OP_CAP = L + 2;       // |T| 上限（起点不计）：含终点 1 + drone 1 + truck-only ≤ L

    Timer timer;

    // ============== Pass 1：D_T(S, v, w)，仅当 |S| ≤ L+1 ========================
    vector<double> dt(static_cast<size_t>(N) * n * n, INF);
    auto DT = [&](int mask, int v, int w) -> double& {
        return dt[(static_cast<size_t>(mask) * n + v) * n + w];
    };
    for (int w = 0; w < n; ++w) {
        int mask = 1 << w;
        for (int v = 0; v < n; ++v) if (v != w)
            DT(mask, v, w) = inst.c(v, w);
    }
    for (int mask = 1; mask < N; ++mask) {
        int pc = __builtin_popcount(mask);
        if (pc < 2 || pc > L + 1) continue;
        for (int w = 0; w < n; ++w) {
            if (!(mask & (1 << w))) continue;
            int sub = mask ^ (1 << w);
            for (int v = 0; v < n; ++v) {
                if (mask & (1 << v)) continue;
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

    // ============== Pass 2：D_OP(S, v, w)，仅当 |S| ≤ L+2 =======================
    vector<double> dop(static_cast<size_t>(N) * n * n, INF);
    vector<int>    dop_d(static_cast<size_t>(N) * n * n, -1);
    auto DOP   = [&](int mask, int v, int w) -> double& { return dop  [(size_t)mask * n * n + v * n + w]; };
    auto DOP_D = [&](int mask, int v, int w) -> int&    { return dop_d[(size_t)mask * n * n + v * n + w]; };

    for (int mask = 1; mask < N; ++mask) {
        int pc = __builtin_popcount(mask);
        if (pc > OP_CAP) continue;
        for (int w = 0; w < n; ++w) {
            if (!(mask & (1 << w))) continue;
            for (int v = 0; v < n; ++v) {
                if (mask & (1 << v)) continue;
                if (mask == (1 << w)) {
                    DOP(mask, v, w) = DT(mask, v, w);
                    DOP_D(mask, v, w) = -1;
                    continue;
                }
                double best = INF;
                int    best_d = -1;
                for (int d = 0; d < n; ++d) {
                    if (d == w || d == v) continue;
                    if (!(mask & (1 << d))) continue;
                    int sub = mask ^ (1 << d);
                    double drone_t = inst.cD(v, d) + inst.cD(d, w);
                    if (drone_t > inst.drone_endurance) continue;
                    double truck_t = DT(sub, v, w);
                    if (truck_t >= INF) continue;
                    double op_t = std::max(drone_t, truck_t);
                    if (op_t < best) { best = op_t; best_d = d; }
                }
                DOP(mask, v, w) = best;
                DOP_D(mask, v, w) = best_d;
            }
        }
    }

    // ============== Pass 3：D(S, w) ============================================
    vector<double> D (static_cast<size_t>(N) * n, INF);
    vector<int>    pre_u(static_cast<size_t>(N) * n, -1);
    vector<int>    pre_T(static_cast<size_t>(N) * n, 0);
    auto Dx   = [&](int mask, int w) -> double& { return D    [(size_t)mask * n + w]; };
    auto PreU = [&](int mask, int w) -> int&    { return pre_u[(size_t)mask * n + w]; };
    auto PreT = [&](int mask, int w) -> int&    { return pre_T[(size_t)mask * n + w]; };

    Dx(1 << depot, depot) = 0;
    vector<int> masks_by_pc[32];
    for (int m = 1; m < N; ++m) masks_by_pc[__builtin_popcount(m)].push_back(m);

    for (int pc = 2; pc <= n; ++pc) {
        for (int S : masks_by_pc[pc]) {
            if (!(S & (1 << depot))) continue;
	      std::vector<int> ws;
	      for (int w = 0; w < n; ++w) if (S & (1 << w)) ws.push_back(w);
	      if (S == ((1 << n) - 1)) { auto it = std::find(ws.begin(), ws.end(), depot); if (it != ws.end()) { ws.erase(it); ws.push_back(depot); } }
	      for (int w : ws) {
                if (!(S & (1 << w))) continue;
                if (w == depot && S != ((1 << n) - 1)) continue;
                int T_base = S ^ (1 << w);
                for (int Tp = T_base; ; Tp = (Tp - 1) & T_base) {
                    int T = Tp | (1 << w);
                    if (__builtin_popcount(T) > OP_CAP) {       // 限制 |T| ≤ L+2
                        if (Tp == 0) break;
                        continue;
                    }
                    int S_pre = (S & ~T) | (1 << depot);
                    int u_cand = S_pre & ~T;
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

    int FULL = (1 << n) - 1;
    Solution sol;
    sol.obj = Dx(FULL, depot);
    sol.status = (sol.obj >= INF) ? "INFEASIBLE" : "FEASIBLE";  // 启发式不保证 OPTIMAL

    // 解恢复（同 dp-tspd）
    if (sol.obj < INF) {
        vector<std::tuple<int,int,int>> ops_rev;
        int cur_S = FULL, cur_w = depot;
        while (cur_S != (1 << depot)) {
            int u = PreU(cur_S, cur_w);
            int T = PreT(cur_S, cur_w);
            if (u < 0) break;
            ops_rev.emplace_back(u, cur_w, T);
            cur_S = (cur_S & ~T) | (1 << depot);
            cur_w = u;
        }
        std::reverse(ops_rev.begin(), ops_rev.end());
        sol.truck_route.push_back(depot);
        for (auto& [u, w, T] : ops_rev) {
            int d = DOP_D(T, u, w);
            if (d >= 0) {
                int truck_set = T ^ (1 << d);
                vector<int> tail;
                int Smask = truck_set, end = w;
                while (true) {
                    tail.push_back(end);
                    int sub2 = Smask ^ (1 << end);
                    if (sub2 == 0) break;
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
                std::reverse(tail.begin(), tail.end());
                for (int x : tail) sol.truck_route.push_back(x);
                sol.sorties.push_back({u, d, w});
            } else {
                sol.truck_route.push_back(w);
            }
        }
    }

    long long elapsed = timer.ms();
    write_result(args.output, inst, "rdp", L, sol, elapsed);
    std::printf("rdp-tspd L=%d  instance=%s  n=%d  obj=%.4f  time_ms=%lld\n",
                L, inst.name.c_str(), n, sol.obj, elapsed);
    return 0;
}
