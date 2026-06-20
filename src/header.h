// =============================================================================
//  header.h  ——  TSP-D（带无人机的旅行商问题）公共类型与 IO
//
//  对应文献：Bouman, Agatz, Schmidt. Dynamic programming approaches for the
//            traveling salesman problem with drone. Networks, 2018.
//
//  约定：
//    n           = 位置总数（含 depot），depot 编号固定为 0，客户为 1..n-1
//    pos[i]      = (x_i, y_i) 二维坐标
//    truck_cost(i,j) = 卡车从 i 到 j 的时间 = 欧氏距离 / truck_speed
//    drone_cost(i,j) = 无人机从 i 到 j 的时间 = 欧氏距离 / drone_speed
//    速度比 α    = drone_speed / truck_speed （论文中 α 取 2）
//
//  位运算约定：
//    用 32-bit 整数表示子集 S，bit i = 1 表示位置 i ∈ S
//    例如 S = 0b1011 表示 {0, 1, 3}
// =============================================================================
#ifndef TSPD_HEADER_H
#define TSPD_HEADER_H

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <chrono>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <limits>
#include <algorithm>
#include <tuple>

using std::vector;
using std::string;

constexpr double INF = 1e18;

// ---- 实例数据 ---------------------------------------------------------------
struct Instance {
    int n;                          // 位置数（含 depot）
    double truck_speed;
    double drone_speed;
    double drone_endurance;         // 无人机续航（按时间单位）
    vector<double> x, y;            // 坐标
    string name;                    // 实例名（用于结果文件命名）

    // 卡车 / 无人机代价矩阵（一维存储，c[i*n+j]）
    vector<double> ct, cd;

    void build_costs() {
        ct.assign(n * n, 0.0);
        cd.assign(n * n, 0.0);
        for (int i = 0; i < n; ++i)
            for (int j = 0; j < n; ++j) {
                double d = std::hypot(x[i] - x[j], y[i] - y[j]);
                ct[i * n + j] = d / truck_speed;
                cd[i * n + j] = d / drone_speed;
            }
    }
    inline double c (int i, int j) const { return ct[i * n + j]; }
    inline double cD(int i, int j) const { return cd[i * n + j]; }
};

// ---- 解结构 -----------------------------------------------------------------
struct Sortie {                     // 一次无人机出动
    int launch;                     // 起点（卡车上）
    int customer;                   // 无人机访问的客户
    int rendezvous;                 // 终点（与卡车汇合）
};

struct Solution {
    double obj = INF;               // 总时长
    vector<int> truck_route;        // 卡车依次经过的节点（含起讫 depot）
    vector<Sortie> sorties;         // 全部无人机出动
    string status = "UNKNOWN";      // OPTIMAL / FEASIBLE / INFEASIBLE
};

// ---- 输入文件读取 -----------------------------------------------------------
//   格式（# 为注释，可忽略空行）：
//     n=14
//     truck_speed=1.0
//     drone_speed=2.0
//     drone_endurance=1e9
//     0  x0 y0
//     1  x1 y1
//     ...
inline bool load_instance(const string& path, Instance& inst) {
    std::ifstream fin(path);
    if (!fin) { std::fprintf(stderr, "open failed: %s\n", path.c_str()); return false; }
    inst.n = 0;
    inst.truck_speed = 1.0;
    inst.drone_speed = 2.0;
    inst.drone_endurance = 1e18;
    inst.x.clear(); inst.y.clear();

    // 取文件名（去掉路径与扩展名）作为实例名
    size_t s = path.find_last_of("/\\");
    size_t d = path.find_last_of('.');
    inst.name = path.substr(s == string::npos ? 0 : s + 1,
                            (d == string::npos ? path.size() : d) -
                            (s == string::npos ? 0 : s + 1));

    string line;
    while (std::getline(fin, line)) {
        if (line.empty() || line[0] == '#') continue;
        auto eq = line.find('=');
        if (eq != string::npos) {
            string key = line.substr(0, eq);
            string val = line.substr(eq + 1);
            if      (key == "n")               inst.n = std::stoi(val);
            else if (key == "truck_speed")     inst.truck_speed = std::stod(val);
            else if (key == "drone_speed")     inst.drone_speed = std::stod(val);
            else if (key == "drone_endurance") inst.drone_endurance = std::stod(val);
        } else {
            std::istringstream iss(line);
            int id; double x, y;
            if (iss >> id >> x >> y) {
                if ((int)inst.x.size() <= id) {
                    inst.x.resize(id + 1, 0);
                    inst.y.resize(id + 1, 0);
                }
                inst.x[id] = x;
                inst.y[id] = y;
            }
        }
    }
    if ((int)inst.x.size() != inst.n) {
        std::fprintf(stderr, "n=%d but read %zu coords\n", inst.n, inst.x.size());
        return false;
    }
    inst.build_costs();
    return true;
}

// ---- 结果文件写出 -----------------------------------------------------------
//   两块：[summary] 一行汇总（便于 grep）；[solution] 详细路径
inline void write_result(const string& path, const Instance& inst,
                         const string& algo, int L_or_neg1,
                         const Solution& sol, long long time_ms) {
    std::ofstream fout(path);
    fout << "[summary]\n";
    fout << "instance=" << inst.name
         << "  algo="   << algo
         << "  n="      << inst.n
         << "  L="      << (L_or_neg1 < 0 ? string("NA") : std::to_string(L_or_neg1))
         << "  obj="    << sol.obj
         << "  time_ms=" << time_ms
         << "  status=" << sol.status << "\n\n";

    fout << "[solution]\n";
    fout << "truck_route:";
    for (int v : sol.truck_route) fout << ' ' << v;
    fout << "\ndrone_sorties:\n";
    for (auto& s : sol.sorties)
        fout << "    launch=" << s.launch
             << "  customer=" << s.customer
             << "  rendezvous=" << s.rendezvous << "\n";
    fout << "total_cost: " << sol.obj << "\n";
    fout.close();
}

// ---- 计时 -------------------------------------------------------------------
struct Timer {
    std::chrono::steady_clock::time_point t0;
    Timer() : t0(std::chrono::steady_clock::now()) {}
    long long ms() const {
        return std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - t0).count();
    }
};

// ---- 命令行参数解析 ---------------------------------------------------------
struct CliArgs {
    string input, output;
    int L = -1;                     // 仅 restricted DP 使用
};
inline bool parse_args(int argc, char** argv, CliArgs& a) {
    for (int i = 1; i < argc; ++i) {
        string s = argv[i];
        if      (s == "--input"  && i+1 < argc) a.input  = argv[++i];
        else if (s == "--output" && i+1 < argc) a.output = argv[++i];
        else if (s == "--L"      && i+1 < argc) a.L      = std::stoi(argv[++i]);
        else if (s == "--help" || s == "-h") {
            std::printf("Usage: %s --input <file> --output <file> [--L <k>]\n", argv[0]);
            return false;
        }
    }
    if (a.input.empty() || a.output.empty()) {
        std::fprintf(stderr, "missing --input or --output\n");
        return false;
    }
    return true;
}

#endif // TSPD_HEADER_H
