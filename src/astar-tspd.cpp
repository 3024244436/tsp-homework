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
//  L 参数（--L <int>）：
//    无 --L 或 --L -1 ：无限制 A* (k=∞)，保证 OPTIMAL
//    --L N (N≥0) ：限制每次 operation 中 truck-only 节点数 ≤ N
//                  前两遍同 rdp-tspd 做限制，第三遍后继也只枚举 |T|≤N+2
//                  status 为 FEASIBLE（限制可能排除最优解）
//
//  调用：
//    ./astar-tspd --input <data.txt> --output <result.txt>              # 无限制
//    ./astar-tspd --input <data.txt> --output <result.txt> --L 2        # k=2
// =============================================================================
#include "header.h"
#include <queue>

// -------- MST：在子图（按位掩码 node_set 选的节点）上用 Prim 算 MST --------
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
    const bool restricted = (args.L >= 0);
    const int OP_CAP = restricted ? (args.L + 2) : (n + 5);   // 无限制时设很大

    Timer total_timer;
    Timer pass_timer;
    long long pass1_ms = 0, pass2_ms = 0, pass3_ms = 0;

    // ============== Pass 1：D_T ==============================================
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
        if (pc < 2) continue;
        if (restricted && pc > args.L + 1) continue;   // 限制 |S| ≤ L+1
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
    pass1_ms = pass_timer.ms();
    pass_timer = Timer();

    // ============== Pass 2：D_OP =============================================
    vector<double> dop(static_cast<size_t>(N) * n * n, INF);
    vector<int>    dop_d(static_cast<size_t>(N) * n * n, -1);
    auto DOP   = [&](int mask, int v, int w) -> double& {
        return dop[(static_cast<size_t>(mask) * n + v) * n + w];
    };
    auto DOP_D = [&](int mask, int v, int w) -> int& {
        return dop_d[(static_cast<size_t>(mask) * n + v) * n + w];
    };
    for (int mask = 1; mask < N; ++mask) {
        int pc = __builtin_popcount(mask);
        if (pc > OP_CAP) continue;    // 限制 |T| ≤ L+2
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
    pass2_ms = pass_timer.ms();
    pass_timer = Timer();

    // ============== Pass 3：A* 搜索 ==========================================
    const double alpha = inst.drone_speed / inst.truck_speed;
    const double h_factor = 1.0 / (2.0 + alpha);

    auto h_func = [&](int S, int w) -> double {
        int uncovered = FULL & ~S;
        int node_set = uncovered | (1 << w) | (1 << depot);
        return mst_cost(inst, node_set) * h_factor;
    };

    vector<double> g(static_cast<size_t>(N) * n, INF);
    vector<char>   closed(static_cast<size_t>(N) * n, 0);
    vector<int>    parent_u(static_cast<size_t>(N) * n, -1);
    vector<int>    parent_T(static_cast<size_t>(N) * n, 0);
    auto idx = [&](int S, int w) { return static_cast<size_t>(S) * n + w; };

    using Item = std::tuple<double, int, int>;
    std::priority_queue<Item, vector<Item>, std::greater<Item>> pq;

    int src_S = 1 << depot;
    g[idx(src_S, depot)] = 0;
    pq.emplace(h_func(src_S, depot), src_S, depot);

    // ---- A* 统计 ----
    long long expanded_states = 0;
    long long generated_states = 1;   // 初始状态已加入
    long long max_queue_size = 1;

    bool found = false;
    while (!pq.empty()) {
        auto [f_cur, S, w] = pq.top(); pq.pop();
        size_t id = idx(S, w);
        if (closed[id]) continue;
        closed[id] = 1;
        expanded_states++;

        if (S == FULL && w == depot) { found = true; break; }

        int free_cust = FULL & ~S;

        // ---- 后继 (a)：非末次 op，w' 是客户，T ⊆ free_cust 含 w' ---
        for (int T = free_cust; ; T = (T - 1) & free_cust) {
            if (T != 0) {
                // 限制检查：|T| ≤ OP_CAP
                if (__builtin_popcount(T) > OP_CAP) {
                    if (T == 0) break;
                    continue;
                }
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
                        generated_states++;
                    }
                }
            }
            if (T == 0) break;
        }

        // ---- 后继 (b)：末次 op，w' = depot，T = free_cust ∪ {depot} ----
        if (w != depot) {
            int T_final = free_cust | (1 << depot);
            int tf_pc = __builtin_popcount(T_final);
            if (tf_pc > OP_CAP) {
                // 限制下不允许，跳过
            } else {
                double op = DOP(T_final, w, depot);
                if (op < INF) {
                    double g_new = g[id] + op;
                    size_t id_sink = idx(FULL, depot);
                    if (g_new < g[id_sink]) {
                        g[id_sink] = g_new;
                        parent_u[id_sink] = w;
                        parent_T[id_sink] = T_final;
                        pq.emplace(g_new + h_func(FULL, depot), FULL, depot);
                        generated_states++;
                    }
                }
            }
        }

        // 更新队列最大长度
        if ((long long)pq.size() > max_queue_size)
            max_queue_size = (long long)pq.size();
    }
    pass3_ms = pass_timer.ms();

    // ============== 解恢复 ===================================================
    Solution sol;
    sol.obj = found ? g[idx(FULL, depot)] : INF;
    sol.status = found ? (restricted ? "FEASIBLE" : "OPTIMAL") : "INFEASIBLE";

    if (found) {
        vector<std::tuple<int, int, int>> ops_rev;
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

    long long total_ms = total_timer.ms();
    int L_out = restricted ? args.L : -1;
    write_result(args.output, inst, "astar", L_out, sol, total_ms);

    // ---- 附加 A* 搜索统计（写在结果文件末尾） ----
    {
        std::ofstream fout(args.output, std::ios::app);
        fout << "\n[astar_stats]\n";
        fout << "pass1_ms=" << pass1_ms << "\n";
        fout << "pass2_ms=" << pass2_ms << "\n";
        fout << "pass3_ms=" << pass3_ms << "\n";
        fout << "expanded_states=" << expanded_states << "\n";
        fout << "generated_states=" << generated_states << "\n";
        fout << "max_queue_size=" << max_queue_size << "\n";
        fout.close();
    }

    std::printf("astar-tspd  L=%s  instance=%s  n=%d  obj=%.4f  time_ms=%lld  "
                "expanded=%lld  generated=%lld  P1=%lld P2=%lld P3=%lld\n",
                restricted ? std::to_string(args.L).c_str() : "inf",
                inst.name.c_str(), n, sol.obj, total_ms,
                expanded_states, generated_states,
                pass1_ms, pass2_ms, pass3_ms);
    return 0;
}
