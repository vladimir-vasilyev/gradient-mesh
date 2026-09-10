#include "gmcore/ContourTracing.h"

#include <algorithm>
#include <cmath>
#include <queue>

namespace gmcore {

namespace {

inline bool fg(const std::vector<uint8_t>& mask, int w, int h, int x, int y) {
    return x >= 0 && x < w && y >= 0 && y < h && mask[y * w + x] != 0;
}

// Isolates the largest 8-connected foreground component of `mask` into
// its own same-size 0/1 mask via BFS flood fill, trying every unvisited
// foreground pixel as a potential component seed (raster order) and
// keeping the biggest. A real graph-cut result can leave small stray
// islands (noise, a same-colored patch far from the scribbles); tracing
// the whole mask as-is would produce a self-intersecting mess, so this
// keeps only the one component actually meant as "the object".
std::vector<uint8_t> largestComponentMask(const std::vector<uint8_t>& mask, int w, int h) {
    std::vector<uint8_t> visited(mask.size(), 0);
    std::vector<int> bestComponent;
    size_t bestSize = 0;
    static const int dx8[8] = {1, 1, 0, -1, -1, -1, 0, 1};
    static const int dy8[8] = {0, 1, 1, 1, 0, -1, -1, -1};

    for (int y = 0; y < h; ++y) {
        for (int x = 0; x < w; ++x) {
            int start = y * w + x;
            if (!mask[start] || visited[start]) continue;
            std::vector<int> component;
            std::queue<int> q;
            q.push(start);
            visited[start] = 1;
            while (!q.empty()) {
                int idx = q.front();
                q.pop();
                component.push_back(idx);
                int cx = idx % w, cy = idx / w;
                for (int n = 0; n < 8; ++n) {
                    int nx = cx + dx8[n], ny = cy + dy8[n];
                    if (!fg(mask, w, h, nx, ny)) continue;
                    int nidx = ny * w + nx;
                    if (visited[nidx]) continue;
                    visited[nidx] = 1;
                    q.push(nidx);
                }
            }
            if (component.size() > bestSize) {
                bestSize = component.size();
                bestComponent = std::move(component);
            }
        }
    }

    std::vector<uint8_t> result(mask.size(), 0);
    for (int idx : bestComponent) result[idx] = 1;
    return result;
}

double perpendicularDistance(const Vec2& p, const Vec2& a, const Vec2& b) {
    double dx = b.x - a.x, dy = b.y - a.y;
    double lenSq = dx * dx + dy * dy;
    if (lenSq < 1e-12) {
        double ex = p.x - a.x, ey = p.y - a.y;
        return std::sqrt(ex * ex + ey * ey);
    }
    // |cross(b-a, a-p)| / |b-a|
    double cross = dx * (a.y - p.y) - dy * (a.x - p.x);
    return std::fabs(cross) / std::sqrt(lenSq);
}

// Standard recursive Douglas-Peucker over an OPEN chain points[from..to]
// (inclusive indices into the shared `points` array) -- marks points to
// KEEP into `keep` (already sized to points.size(), with points[from] and
// points[to] expected to already be marked true by the caller).
void douglasPeuckerRecurse(const std::vector<Vec2>& points, int from, int to, double epsilon,
                            std::vector<uint8_t>& keep) {
    if (to <= from + 1) return;
    double maxDist = -1.0;
    int maxIdx = -1;
    for (int i = from + 1; i < to; ++i) {
        double d = perpendicularDistance(points[i], points[from], points[to]);
        if (d > maxDist) { maxDist = d; maxIdx = i; }
    }
    if (maxDist > epsilon && maxIdx >= 0) {
        keep[maxIdx] = 1;
        douglasPeuckerRecurse(points, from, maxIdx, epsilon, keep);
        douglasPeuckerRecurse(points, maxIdx, to, epsilon, keep);
    }
}

} // namespace

std::vector<Vec2> traceOuterContour(const std::vector<uint8_t>& mask, int width, int height) {
    if (width <= 0 || height <= 0) return {};
    std::vector<uint8_t> component = largestComponentMask(mask, width, height);

    // Moore-neighbor tracing, clockwise neighbor order starting at East
    // (image coordinates, y increasing downward -- see MaxFlowGraph-
    // adjacent comment style: this ordering IS visually clockwise on
    // screen with that axis convention).
    static const int dx8[8] = {1, 1, 0, -1, -1, -1, 0, 1};
    static const int dy8[8] = {0, 1, 1, 1, 0, -1, -1, -1};

    int startX = -1, startY = -1;
    for (int y = 0; y < height && startX < 0; ++y) {
        for (int x = 0; x < width; ++x) {
            if (component[y * width + x]) { startX = x; startY = y; break; }
        }
    }
    if (startX < 0) return {}; // no foreground pixels at all

    std::vector<Vec2> boundary;
    boundary.push_back(Vec2{(double)startX, (double)startY});

    int curX = startX, curY = startY;
    int backtrackDir = 4; // West -- s0's west neighbor is guaranteed background,
                           // since s0 is the first foreground pixel in raster order

    // Gonzalez & Woods' classic Moore-boundary-tracing stopping rule: NOT
    // "we're back at s0 having arrived via the same backtrack direction
    // we started with" (that direction is an artifact of how s0's search
    // was seeded, and the trace can legitimately re-enter s0 later from a
    // DIFFERENT real direction -- e.g. a filled square's trace returns to
    // its top-left corner from the north, not the west it was seeded
    // with, and comparing backtrack directions there never matches,
    // spinning forever; caught by test_contour_tracing_square returning
    // ~866 points for a 5x5 square instead of ~20). The correct rule
    // instead remembers s1, the SECOND boundary point ever found, and
    // stops when the trace is about to re-find s1 immediately after
    // being at s0 again -- i.e. it's about to repeat the very first
    // transition, regardless of which direction it re-arrived at s0 from.
    bool haveS1 = false;
    int s1X = 0, s1Y = 0;

    // Generous safety cap: a simple closed 8-connected boundary visits
    // each pixel at most a small constant number of times in practice;
    // this is far above any real contour length and just guards against
    // an unforeseen edge case looping forever.
    const long long maxSteps = 8LL * (long long)width * height + 64;
    long long steps = 0;

    while (true) {
        int startSearchDir = (backtrackDir + 1) % 8;
        int foundDir = -1;
        for (int i = 0; i < 8; ++i) {
            int dir = (startSearchDir + i) % 8;
            int nx = curX + dx8[dir], ny = curY + dy8[dir];
            if (fg(component, width, height, nx, ny)) { foundDir = dir; break; }
        }
        if (foundDir < 0) break; // isolated single pixel -- nothing to trace onward

        int nx = curX + dx8[foundDir], ny = curY + dy8[foundDir];

        if (!haveS1) {
            haveS1 = true;
            s1X = nx; s1Y = ny;
        } else if (curX == startX && curY == startY && nx == s1X && ny == s1Y) {
            break; // about to repeat the s0->s1 transition: the loop is closed
        }

        backtrackDir = (foundDir + 4) % 8;
        curX = nx;
        curY = ny;
        boundary.push_back(Vec2{(double)curX, (double)curY});

        if (++steps > maxSteps) break; // defensive; should not trigger on a valid simple contour
    }

    return boundary;
}

std::vector<Vec2> simplifyClosedPolygon(const std::vector<Vec2>& points, double epsilonPixels) {
    int n = (int)points.size();
    if (n < 4) return points; // nothing meaningful to simplify

    // Split the closed loop into two open chains at index 0 and index
    // n/2 -- a simple, standard choice (not necessarily the true diameter
    // pair, but close enough for a roughly blob-shaped segmentation
    // contour, and far cheaper than an all-pairs search).
    int split = n / 2;
    std::vector<uint8_t> keep(n, 0);
    keep[0] = 1;
    keep[split] = 1;

    douglasPeuckerRecurse(points, 0, split, epsilonPixels, keep);
    // Second chain wraps from `split` back to `0` (through n-1) -- build
    // an explicit index sequence so douglasPeuckerRecurse can treat it as
    // a normal contiguous range.
    std::vector<int> wrapIndices;
    for (int i = split; i < n; ++i) wrapIndices.push_back(i);
    wrapIndices.push_back(0); // close back to the start point
    std::vector<Vec2> wrapPoints;
    wrapPoints.reserve(wrapIndices.size());
    for (int idx : wrapIndices) wrapPoints.push_back(points[idx]);
    std::vector<uint8_t> wrapKeep(wrapPoints.size(), 0);
    wrapKeep[0] = 1;
    wrapKeep[wrapKeep.size() - 1] = 1;
    douglasPeuckerRecurse(wrapPoints, 0, (int)wrapPoints.size() - 1, epsilonPixels, wrapKeep);
    for (size_t i = 0; i < wrapKeep.size(); ++i) {
        if (wrapKeep[i]) keep[wrapIndices[i]] = 1;
    }

    std::vector<Vec2> result;
    for (int i = 0; i < n; ++i) {
        if (keep[i]) result.push_back(points[i]);
    }
    return result;
}

} // namespace gmcore
