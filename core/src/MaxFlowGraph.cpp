#include "gmcore/MaxFlowGraph.h"

#include <algorithm>
#include <limits>
#include <queue>

namespace gmcore {

namespace {
constexpr double kEps = 1e-12; // treat residual capacity below this as zero
}

MaxFlowGraph::MaxFlowGraph(int numNodes) : adj_(numNodes), numNodes_(numNodes) {}

int MaxFlowGraph::addEdge(int u, int v, double cap, double reverseCap) {
    int idx = (int)edges_.size();
    edges_.push_back({v, cap});
    edges_.push_back({u, reverseCap});
    adj_[u].push_back(idx);
    adj_[v].push_back(idx + 1);
    return idx;
}

bool MaxFlowGraph::bfs(int source, int sink) {
    level_.assign(numNodes_, -1);
    std::queue<int> q;
    level_[source] = 0;
    q.push(source);
    while (!q.empty()) {
        int u = q.front();
        q.pop();
        for (int eid : adj_[u]) {
            const Edge& e = edges_[eid];
            if (e.cap > kEps && level_[e.to] < 0) {
                level_[e.to] = level_[u] + 1;
                q.push(e.to);
            }
        }
    }
    return level_[sink] >= 0;
}

double MaxFlowGraph::dfs(int u, int sink, double pushed) {
    if (u == sink || pushed <= 0.0) return pushed;
    for (int& i = iter_[u]; i < (int)adj_[u].size(); ++i) {
        int eid = adj_[u][i];
        Edge& e = edges_[eid];
        if (e.cap > kEps && level_[e.to] == level_[u] + 1) {
            double d = dfs(e.to, sink, std::min(pushed, e.cap));
            if (d > 0.0) {
                e.cap -= d;
                edges_[eid ^ 1].cap += d;
                return d;
            }
        }
    }
    return 0.0;
}

double MaxFlowGraph::maxFlow(int source, int sink) {
    double flow = 0.0;
    while (bfs(source, sink)) {
        iter_.assign(numNodes_, 0);
        double pushed;
        while ((pushed = dfs(source, sink, std::numeric_limits<double>::max())) > 0.0) {
            flow += pushed;
        }
    }
    // `level_` now holds the BFS reachability from `source` in the FINAL
    // residual graph (the last bfs() call above, whose false return is
    // what ended the loop) -- exactly what isSourceSide() needs. No extra
    // pass required.
    return flow;
}

bool MaxFlowGraph::isSourceSide(int u) const {
    return u >= 0 && u < (int)level_.size() && level_[u] >= 0;
}

} // namespace gmcore
