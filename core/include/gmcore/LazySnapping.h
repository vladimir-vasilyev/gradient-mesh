// LazySnapping.h — interactive foreground/background segmentation from
// user foreground/background scribbles, via colour-cluster data terms and
// a contrast-sensitive graph cut (MaxFlowGraph.h). This is this project's
// answer to Sun et al.'s citation of Li, Sun & Shum's "Lazy Snapping"
// (SIGGRAPH 2004) as the paper's own cutout tool (see README's "Known
// simplifications" -- this replaces the plain click-tracing polygon tool
// that stood in for it until now).
//
// Energy model (the standard Boykov & Jolly 2001 "Interactive Graph
// Cuts" formulation that Lazy Snapping's own graph-cut step is built on;
// implemented here from that general graph-cuts formulation, not
// re-derived from the original Lazy Snapping paper's exact text -- see
// segmentForeground's own comment in the .cpp for the precise per-term
// formulas and an honest note on that distinction):
//
//   * Data (unary) term per pixel: a small set of representative colours
//     ("cluster centres") is built from each of the foreground/background
//     scribbles via k-means; a pixel's cost of being labelled background
//     is proportional to its colour distance to the nearest BACKGROUND
//     cluster (i.e. cheap/likely to call it background when its colour
//     genuinely matches the background samples), and symmetrically for
//     foreground -- like a -log-likelihood data term under a background
//     vs. foreground colour model. Scribbled pixels themselves get a hard
//     constraint (effectively-infinite cost for the label they weren't
//     scribbled as) instead of the soft cluster-based cost.
//   * Smoothness (pairwise) term over each pixel's 8-connected neighbours:
//     a contrast-sensitive weight that's large where two neighbouring
//     pixels have similar colour (expensive to cut the boundary through a
//     flat region) and small across a strong colour edge (cheap to cut
//     there) -- so the resulting cut boundary is pulled toward real image
//     edges, not just the shortest path between scribbles.
//
// The resulting minimum cut assigns every pixel a foreground/background
// label; see ContourTracing.h for turning that binary mask into the
// ordered boundary polygon CanvasView/DocumentModel's existing corner-
// picking + per-side Bezier fit already knows how to consume (see
// DocumentModel.h's -setBoundaryPolygonPoints:/-fitBoundaryWithCornerIndices:)
// -- Lazy Snapping only replaces how that polygon is OBTAINED, not what
// happens to it afterward.
#pragma once

#include "gmcore/Image.h"

#include <cstdint>
#include <utility>
#include <vector>

namespace gmcore {

// User-drawn scribbles, as pixel coordinates (x,y; not required to be
// unique or ordered -- typically every pixel a brush stroke passed over).
// A pixel scribbled in BOTH lists is treated as foreground (foreground
// wins ties) -- see segmentForeground's own comment.
struct SegmentationScribbles {
    std::vector<std::pair<int, int>> foreground;
    std::vector<std::pair<int, int>> background;
};

struct LazySnappingOptions {
    // Number of k-means cluster centres built from EACH of the
    // foreground/background scribble color sets (see kMeansClusters in
    // the .cpp). Small on purpose -- this is a coarse colour model for a
    // fast interactive tool, not a fitted density estimate; 8 is enough
    // to separate a handful of distinct colour regions per side without
    // needing many scribble strokes to seed it.
    int numColorClusters = 8;
    // Overall weight of the pairwise smoothness term relative to the
    // unary colour-cluster data term (both already normalized to
    // comparable scale internally -- see the .cpp's own comment on the
    // exact scaling). Higher values pull the cut boundary toward being
    // "smoother"/more edge-seeking and less willing to follow the raw
    // per-pixel colour-cluster verdict; lower values track the data term
    // more literally, at the cost of a noisier boundary.
    double smoothnessWeight = 4.0;
    // A hard constraint's edge capacity -- must simply be much larger
    // than any possible sum of soft edge weights in the graph so it can
    // never be the cheapest edge to cut. Not "infinity" (that would
    // break Dinic's floating-point arithmetic) -- see the .cpp for how
    // this is validated against the actual graph size before use.
    double hardConstraintWeight = 1e9;
};

// Runs the graph cut and returns a row-major width*height mask (1 =
// foreground, 0 = background). Returns an all-zero mask (with a
// documented reason -- see the .cpp) if there are no scribbles of BOTH
// colours yet (nothing to cut between).
std::vector<uint8_t> segmentForeground(const Image& image, const SegmentationScribbles& scribbles,
                                        const LazySnappingOptions& opts = {});

} // namespace gmcore
