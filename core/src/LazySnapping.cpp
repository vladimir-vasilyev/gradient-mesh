#include "gmcore/LazySnapping.h"
#include "gmcore/MaxFlowGraph.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <random>

namespace gmcore {

namespace {

double colorDistSq(const Color& a, const Color& b) {
    double dr = a.r - b.r, dg = a.g - b.g, db = a.b - b.b;
    return dr * dr + dg * dg + db * db;
}

// Minimal k-means over Color samples (squared Euclidean sRGB distance).
// Deliberately not a general-purpose clustering utility -- just enough to
// build LazySnapping's small per-region "representative colour set"; not
// declared in the header. Deterministic (fixed seed, evenly-spaced
// initial centres from the sorted-by-luma sample list) rather than a
// random restart scheme, so a given scribble set always segments the same
// way -- important for an interactive tool the user is iterating on.
std::vector<Color> kMeansClusters(const std::vector<Color>& samples, int k, int iterations = 12) {
    if (samples.empty()) return {};
    k = std::min<int>(k, (int)samples.size());
    if (k <= 0) return {};

    // Seed centres by luma-sorted even spacing rather than the first k
    // samples (which could all be near-duplicates if the scribble stroke
    // lingered in one spot) -- a cheap, deterministic stand-in for
    // k-means++'s spread-out seeding.
    std::vector<Color> sorted = samples;
    std::sort(sorted.begin(), sorted.end(), [](const Color& a, const Color& b) {
        return (a.r + a.g + a.b) < (b.r + b.g + b.b);
    });
    std::vector<Color> centres(k);
    for (int i = 0; i < k; ++i) {
        size_t idx = sorted.size() == 1 ? 0 : (size_t)((double)i * (sorted.size() - 1) / std::max(1, k - 1));
        centres[i] = sorted[idx];
    }

    std::vector<int> assignment(samples.size(), 0);
    for (int iter = 0; iter < iterations; ++iter) {
        bool changed = false;
        for (size_t i = 0; i < samples.size(); ++i) {
            double best = std::numeric_limits<double>::max();
            int bestK = 0;
            for (int c = 0; c < k; ++c) {
                double d = colorDistSq(samples[i], centres[c]);
                if (d < best) { best = d; bestK = c; }
            }
            if (assignment[i] != bestK) { assignment[i] = bestK; changed = true; }
        }
        std::vector<Color> sum(k, Color{0, 0, 0});
        std::vector<int> count(k, 0);
        for (size_t i = 0; i < samples.size(); ++i) {
            sum[assignment[i]] += samples[i];
            count[assignment[i]]++;
        }
        for (int c = 0; c < k; ++c) {
            if (count[c] > 0) centres[c] = sum[c] / (double)count[c];
        }
        if (!changed) break;
    }

    // Drop clusters that ended up with no members (possible if k exceeds
    // the number of genuinely distinct colours in `samples`).
    std::vector<Color> result;
    std::vector<int> finalCount(k, 0);
    for (int a : assignment) finalCount[a]++;
    for (int c = 0; c < k; ++c) {
        if (finalCount[c] > 0) result.push_back(centres[c]);
    }
    return result;
}

double distToNearestCluster(const Color& c, const std::vector<Color>& clusters) {
    double best = std::numeric_limits<double>::max();
    for (const Color& cl : clusters) best = std::min(best, colorDistSq(c, cl));
    return best;
}

} // namespace

std::vector<uint8_t> segmentForeground(const Image& image, const SegmentationScribbles& scribbles,
                                        const LazySnappingOptions& opts) {
    const int w = image.width, h = image.height;
    std::vector<uint8_t> mask(std::max(0, w * h), 0);
    if (w <= 0 || h <= 0) return mask;

    if (scribbles.foreground.empty() || scribbles.background.empty()) {
        // Nothing to cut between -- an all-background mask is the honest
        // "not enough information yet" answer, not a guess.
        return mask;
    }

    auto inBounds = [&](int x, int y) { return x >= 0 && x < w && y >= 0 && y < h; };

    // Foreground scribbles win ties (a pixel scribbled both ways) --
    // applied by marking foreground into a lookup AFTER background, so
    // it overwrites.
    std::vector<int8_t> scribbleLabel(w * h, -1); // -1 unscribbled, 0 bg, 1 fg
    for (auto& p : scribbles.background)
        if (inBounds(p.first, p.second)) scribbleLabel[p.second * w + p.first] = 0;
    for (auto& p : scribbles.foreground)
        if (inBounds(p.first, p.second)) scribbleLabel[p.second * w + p.first] = 1;

    // --- Colour cluster models (data term) ---
    std::vector<Color> fgSamples, bgSamples;
    fgSamples.reserve(scribbles.foreground.size());
    bgSamples.reserve(scribbles.background.size());
    for (auto& p : scribbles.foreground)
        if (inBounds(p.first, p.second)) fgSamples.push_back(image.at(p.first, p.second));
    for (auto& p : scribbles.background)
        if (inBounds(p.first, p.second)) bgSamples.push_back(image.at(p.first, p.second));
    std::vector<Color> fgClusters = kMeansClusters(fgSamples, opts.numColorClusters);
    std::vector<Color> bgClusters = kMeansClusters(bgSamples, opts.numColorClusters);
    if (fgClusters.empty() || bgClusters.empty()) return mask; // scribbles landed out of bounds

    // --- Smoothness term's contrast sensitivity: auto-estimate sigma^2
    // as the mean squared colour distance between all 8-connected
    // neighbour pairs (the standard Boykov-Jolly auto-tuning heuristic --
    // ties the "how much local contrast counts as an edge" threshold to
    // THIS image's own actual contrast statistics instead of a fixed
    // magic constant). ---
    double sumSq = 0.0;
    long long count = 0;
    static const int kNbrDx[4] = {1, 0, 1, 1};
    static const int kNbrDy[4] = {0, 1, 1, -1};
    for (int y = 0; y < h; ++y) {
        for (int x = 0; x < w; ++x) {
            Color c0 = image.at(x, y);
            for (int n = 0; n < 4; ++n) { // only "forward" neighbours -- each pair counted once
                int nx = x + kNbrDx[n], ny = y + kNbrDy[n];
                if (!inBounds(nx, ny)) continue;
                sumSq += colorDistSq(c0, image.at(nx, ny));
                ++count;
            }
        }
    }
    double sigmaSq = count > 0 ? std::max(sumSq / (double)count, 1e-6) : 1e-6;

    // --- Build the graph: one node per pixel, plus source (=fg terminal)
    // and sink (=bg terminal). ---
    const int source = w * h;
    const int sink = w * h + 1;
    MaxFlowGraph graph(w * h + 2);

    // Terminal (unary/data) edges -- see this file's header comment for
    // the capacity<->label-cost convention and its derivation:
    //   capacity(source, p) = cost of labelling p BACKGROUND
    //                       ~ colour distance to nearest FOREGROUND cluster
    //   capacity(p, sink)   = cost of labelling p FOREGROUND
    //                       ~ colour distance to nearest BACKGROUND cluster
    // (cutting the source->p edge removes p from the source/foreground
    // side, so that edge's cost must be what a BACKGROUND label costs --
    // which should be LOW when p looks nothing like the foreground
    // samples, i.e. proportional to distance from the foreground
    // clusters; symmetric reasoning for p->sink.)
    for (int y = 0; y < h; ++y) {
        for (int x = 0; x < w; ++x) {
            int node = y * w + x;
            int8_t lbl = scribbleLabel[node];
            double toSource, toSink;
            if (lbl == 1) {           // scribbled foreground: hard-constrained
                toSource = opts.hardConstraintWeight;
                toSink = 0.0;
            } else if (lbl == 0) {    // scribbled background: hard-constrained
                toSource = 0.0;
                toSink = opts.hardConstraintWeight;
            } else {
                // capacity(source,p) is the edge severed when p ends up
                // BACKGROUND (source-unreachable) -- so it must equal the
                // COST of that label, D_p(background), which by
                // definition is LOW when p's colour matches the
                // background model well (i.e. proportional to distance
                // from the BACKGROUND clusters, not the foreground ones
                // -- an earlier version of this had fg/bg swapped here,
                // caught by test_segmentation_separates_color_blocks
                // returning an almost-empty mask instead of splitting the
                // image in two: with the swap, every pixel's D_p(bkg) was
                // its distance to the FOREGROUND clusters, i.e. ~0 deep
                // inside the true foreground region -- making it cheap to
                // cut EVERY non-scribbled foreground pixel to background,
                // which is exactly the "only the literal scribble pixels
                // survive" failure that test caught).
                Color c = image.at(x, y);
                toSource = distToNearestCluster(c, bgClusters); // D_p(background)
                toSink = distToNearestCluster(c, fgClusters);   // D_p(foreground)
            }
            graph.addEdge(source, node, toSource);
            graph.addEdge(node, sink, toSink);
        }
    }

    // Pairwise (smoothness) edges -- 8-connected, "forward" neighbours
    // only (each unordered pair added once, as an undirected edge: equal
    // capacity both ways, since the Potts penalty for landing on
    // different sides of the cut doesn't depend on which side is which).
    // 1/dist(p,q) weakens the diagonal neighbours slightly relative to
    // the axis ones (dist sqrt(2) vs 1), the standard correction so an
    // 8-connected grid's cut-boundary "cost per unit length" doesn't
    // depend on the boundary's own orientation.
    for (int y = 0; y < h; ++y) {
        for (int x = 0; x < w; ++x) {
            int node = y * w + x;
            Color c0 = image.at(x, y);
            for (int n = 0; n < 4; ++n) {
                int nx = x + kNbrDx[n], ny = y + kNbrDy[n];
                if (!inBounds(nx, ny)) continue;
                double dist = (kNbrDx[n] != 0 && kNbrDy[n] != 0) ? 1.4142135623730951 : 1.0;
                double d2 = colorDistSq(c0, image.at(nx, ny));
                double weight = opts.smoothnessWeight * std::exp(-d2 / (2.0 * sigmaSq)) / dist;
                int neighborNode = ny * w + nx;
                graph.addEdge(node, neighborNode, weight, weight);
            }
        }
    }

    graph.maxFlow(source, sink);
    for (int i = 0; i < w * h; ++i) {
        mask[i] = graph.isSourceSide(i) ? 1 : 0;
    }
    return mask;
}

} // namespace gmcore
