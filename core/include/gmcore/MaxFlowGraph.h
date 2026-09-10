// MaxFlowGraph.h — Dinic's maximum-flow algorithm over a directed graph
// with non-negative real edge capacities.
//
// This is the low-level, segmentation-agnostic building block for
// LazySnapping.h/.cpp's interactive foreground/background graph-cut
// segmentation (see that file): by max-flow/min-cut duality, a minimum
// s-t cut's total capacity equals the maximum s-t flow, and the set of
// nodes still reachable from the source in the FINAL residual graph (see
// isSourceSide()) IS one minimum cut -- exactly the foreground/background
// split Lazy Snapping needs. This class knows nothing about images,
// pixels, or colour.
//
// Dinic's algorithm was chosen over the closely related Boykov-Kolmogorov
// algorithm (the other standard choice for interactive image
// segmentation) deliberately: BK's main advantage over Dinic's is
// reusing its search trees to warm-start a new solve when only a few
// terminal weights change, which matters for a UI that re-solves after
// every small scribble edit. This project's UI instead re-runs the solve
// once per "Segment" click on a complete, fixed set of scribbles, so that
// advantage doesn't apply here -- and Dinic's textbook level-graph +
// blocking-flow structure is meaningfully simpler to implement and verify
// correctly from scratch, which matters more for a from-scratch
// implementation like this project's. Worst case is O(V^2 * E), but the
// well-known result for unit-capacity/grid-like graphs -- and the
// empirical result for 2D image grid-cut graphs specifically, well
// established since Boykov & Jolly's 2001 "Interactive Graph Cuts" paper
// -- is that it runs close to O(E) in practice on exactly this kind of
// graph, which is why Dinic's/BK are the standard choices for interactive
// segmentation rather than a generic max-flow algorithm.
#pragma once

#include <vector>

namespace gmcore {

class MaxFlowGraph {
public:
    explicit MaxFlowGraph(int numNodes);

    // Adds a directed edge u->v with capacity `cap`, and its paired
    // reverse residual edge v->u with capacity `reverseCap` (0 by
    // default, i.e. a normal one-way flow-network edge). Pass
    // reverseCap == cap for an "undirected" edge -- LazySnapping's
    // pixel-neighbour smoothness edges do this, since the Potts-style
    // smoothness penalty is the same regardless of which side of the cut
    // ends up foreground. Returns the new edge's index; its paired
    // reverse edge is always at index^1 (standard even/odd residual-edge
    // bookkeeping).
    int addEdge(int u, int v, double cap, double reverseCap = 0.0);

    // Computes the maximum flow from `source` to `sink` via Dinic's
    // algorithm (repeated BFS level-graph construction + DFS blocking
    // flow, until the sink is no longer reachable in the level graph) and
    // returns its value. Mutates internal residual capacities -- call at
    // most once per graph; build a fresh MaxFlowGraph for a different
    // scribble set rather than reusing one.
    double maxFlow(int source, int sink);

    // Valid only after maxFlow() has returned. True if node `u` is still
    // reachable from `source` via strictly-positive-residual-capacity
    // edges in the FINAL residual graph -- i.e. `u` sits on the SOURCE
    // side of a minimum s-t cut (max-flow/min-cut duality guarantees the
    // source-reachable set at termination is exactly one minimum cut).
    // LazySnapping calls this once per pixel node to recover the
    // foreground mask.
    bool isSourceSide(int u) const;

    int numNodes() const { return numNodes_; }

private:
    struct Edge { int to; double cap; };
    std::vector<Edge> edges_;
    std::vector<std::vector<int>> adj_; // adj_[u] = indices into edges_
    std::vector<int> level_;            // BFS distance from source; -1 = unreached
    std::vector<int> iter_;             // Dinic's "current arc" pointer per node
    int numNodes_ = 0;

    bool bfs(int source, int sink);
    double dfs(int u, int sink, double pushed);
};

} // namespace gmcore
