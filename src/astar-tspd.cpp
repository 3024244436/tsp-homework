// =============================================================================
//  astar-tspd.cpp  ——  TSP-D 第三遍换成 A* 搜索（论文 §3.4.2）
//
//  与 dp-tspd.cpp 共享前两遍 (Pass 1 / Pass 2)：D_T、D_OP 表的构建完全一样。
//  仅第三遍不同：
//    标准 DP   → 自底向上填表，所有 2^n · n 个状态都算
//    A* 实现   → best-first search，按 f = g + h 优先扩展
//                  g = 已知最优代价 (类似 D 表)
//                  h = MST 下界 / (2 + α)   论文公式见 §3.4.2
//
//  启发式 (heuristic) 的可采纳性：
//    h(S, w) = (1 / (2 + α)) · MST({w} ∪ V̄ ∪ {v_0})
//    其中 V̄ = 未覆盖客户集合
//    由 Agatz et al. [1] 已证："MST 启发式构造的 TSP tour 是 TSP-D 的 (2+α)-近似"
//    故除以 (2 + α) 给出真实剩余代价的下界 → A* 保证找到最优解
//
//  调用：
//    ./astar-tspd --input <data.txt> --output <result.txt>
// =============================================================================
#include "header.h"
#include <queue>

// -------- MST：在子图（按位掩码 node_set 选的节点）上用 Prim 算 MST --------
//   节点数最多 n ≤ 18 左右，O(k^2) 完全够
double mst_cost(const Instance& inst, int node_set) {
    int n = inst.n;
    int k = __builtin_popcount(node_set);
    if (k <= 1) return 0.0;

    vector<int> nodes;
    nodes.reserve(k);
    for (int i = 0; i < n; ++i)
        if (node_set & (1 << i)) nodes.push_back(i);

    vector<double> min_e(k, INF);
    vector<char>   in_tree(k, 0);
    min_e[0] = 0;
    double total = 0;

    for (int iter = 0; iter < k; ++iter) {
        int u = -1;
        double best = INF;
        for (int i = 0; i < k; ++i)
            if (!in_tree[i] && min_e[i] < best) { best = min_e[i]; u = i; }
        if (u < 0) break;
        in_tree[u] = 1;
        total += min_e[u];
        for (int i = 0; i < k; ++i)
            if (!in_tree[i]) {
                double e = inst.c(nodes[u], nodes[i]);
                if (e < min_e[i]) min_e[i] = e;
            }
    }
    return total;
}

int main(int argc, char** argv) {
    CliArgs args;
    if (!parse_args(argc, argv, args)) return 1;

    Instance inst;
    if (!load_instance(args.input, inst)) return 1;
    const int n = inst.n;
    const int N = 1 << n;
    const int depot = 0;
    const int FULL = N - 1;

    Timer timer;

    // ============== Pass 1：D_T（与 dp-tspd.cpp 完全一致） ====================
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
        if (__builtin_popcount(mask) < 2) continue;
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

    // ============== Pass 2：D_OP（与 dp-tspd.cpp 完全一致） ===================
    vector<double> dop(static_cast<size_t>(N) * n * n, INF);
    vector<int>    dop_d(static_cast<size_t>(N) * n * n, -1);
    auto DOP   = [&](int mask, int v, int w) -> double& {
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

    // ============== Pass 3：A* 搜索 ==========================================
    // α = drone_speed / truck_speed（与论文 §2 的 α 定义一致）
    const double alpha = inst.drone_speed / inst.truck_speed;
    const double h_factor = 1.0 / (2.0 + alpha);

    // 启发函数：h(S, w) = MST({w} ∪ uncovered ∪ {depot}) / (2 + α)
    auto h_func = [&](int S, int w) -> double {
        int uncovered = FULL & ~S;
        int node_set = uncovered | (1 << w) | (1 << depot);
        return mst_cost(inst, node_set) * h_factor;
    };

    // g[S*n + w] = 从源到 (S,w) 的已知最短代价
    vector<double> g(static_cast<size_t>(N) * n, INF);
    vector<char>   closed(static_cast<size_t>(N) * n, 0);
    vector<int>    parent_u(static_cast<size_t>(N) * n, -1);
    vector<int>    parent_T(static_cast<size_t>(N) * n, 0);
    auto idx = [&](int S, int w) { return static_cast<size_t>(S) * n + w; };

    // 优先队列：(f, S, w)
    using Item = std::tuple<double, int, int>;
    std::priority_queue<Item, vector<Item>, std::greater<Item>> pq;

    int src_S = 1 << depot;
    g[idx(src_S, depot)] = 0;
    pq.emplace(h_func(src_S, depot), src_S, depot);

    bool found = false;
    while (!pq.empty()) {
        auto [f_cur, S, w] = pq.top(); pq.pop();
        size_t id = idx(S, w);
        if (closed[id]) continue;
        closed[id] = 1;

        if (S == FULL && w == depot) { found = true; break; }   // 到达汇点

        // 扩展所有后继：从 (S, w) 通过一次 operation 转到 (S', w')
        // 自由位 = 未覆盖客户（不含 depot，因为 depot 已在 S 中）
        int free_cust = FULL & ~S;

        // ----- 后继 (a)：非末次 op，w' 是客户，T ⊆ free_cust 含 w' -----
        // 枚举 T ⊆ free_cust（含空集），跳过空集
        for (int T = free_cust; ; T = (T - 1) & free_cust) {
            if (T != 0) {
                // 对 T 中每个位作为 w' 枚举
                int Tb = T;
                while (Tb) {
                    int w_bit = Tb & -Tb;
                    int w_new = __builtin_ctz(w_bit);
                    Tb ^= w_bit;
                    double op = DOP(T, w, w_new);
                    if (op >= INF) continue;
                    int S_new = S | T;
                    double g_new = g[id] + op;
                    size_t id_new = idx(S_new, w_new);
                    if (g_new < g[id_new]) {
                        g[id_new] = g_new;
                        parent_u[id_new] = w;
                        parent_T[id_new] = T;
                        pq.emplace(g_new + h_func(S_new, w_new), S_new, w_new);
                    }
                }
            }
            if (T == 0) break;
        }

        // ----- 后继 (b)：末次 op，w' = depot，T = free_cust ∪ {depot} -----
        if (w != depot) {
            int T_final = free_cust | (1 << depot);
            double op = DOP(T_final, w, depot);
            if (op < INF) {
                double g_new = g[id] + op;
                size_t id_sink = idx(FULL, depot);
                if (g_new < g[id_sink]) {
                    g[id_sink] = g_new;
                    parent_u[id_sink] = w;
                    parent_T[id_sink] = T_final;
                    pq.emplace(g_new + h_func(FULL, depot), FULL, depot);
                }
            }
        }
    }

    // ============== 解恢复（同 dp-tspd） =====================================
    Solution sol;
    sol.obj = found ? g[idx(FULL, depot)] : INF;
    sol.status = found ? "OPTIMAL" : "INFEASIBLE";

    if (found) {
        vector<std::tuple<int, int, int>> ops_rev;          // (u, w, T)
        int cur_S = FULL, cur_w = depot;
        while (cur_S != (1 << depot)) {
            int u = parent_u[idx(cur_S, cur_w)];
            int T = parent_T[idx(cur_S, cur_w)];
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
    write_result(args.output, inst, "astar", -1, sol, elapsed);
    std::printf("astar-tspd  instance=%s  n=%d  obj=%.4f  time_ms=%lld\n",
                inst.name.c_str(), n, sol.obj, elapsed);
    return 0;
}
