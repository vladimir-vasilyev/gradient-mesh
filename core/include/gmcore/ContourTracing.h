// ContourTracing.h — turns a binary foreground/background mask (as
// produced by LazySnapping.h's segmentForeground) into the ordered
// boundary polygon that CanvasView/DocumentModel's EXISTING corner-
// picking + per-side cubic-Bezier fit already knows how to consume (see
// DocumentModel.h's -setBoundaryPolygonPoints:/-fitBoundaryWithCornerIndices:).
// Lazy Snapping only replaces how this polygon is OBTAINED (a graph cut
// instead of manual click-tracing); everything downstream is unchanged.
#pragma once

#include "gmcore/Vec2.h"

#include <cstdint>
#include <vector>

namespace gmcore {

// Traces the outer boundary of the LARGEST 8-connected foreground
// component in `mask` (row-major width*height, nonzero = foreground) via
// Moore-neighbor tracing (Jacob's stopping criterion), returning an
// ordered, closed (first point NOT repeated at the end) polygon of pixel-
// center coordinates. Picks the largest component specifically because a
// real graph-cut result can leave a few stray foreground pixels/islands
// elsewhere in the image (noise, a similarly-colored region far from the
// scribbles) -- those are discarded rather than traced. Returns an empty
// vector if `mask` has no foreground pixels at all.
std::vector<Vec2> traceOuterContour(const std::vector<uint8_t>& mask, int width, int height);

// Ramer-Douglas-Peucker simplification of a CLOSED polygon (as returned
// by traceOuterContour -- pixel-resolution contours are typically
// thousands of points, one per boundary pixel, far more than is useful or
// even clickable for the existing 4-corner-picking UI). Splits the loop
// at two well-separated anchor points, simplifies each resulting open
// chain independently with the standard recursive algorithm, then
// rejoins them -- the standard adaptation of Douglas-Peucker (originally
// defined for open polylines) to a closed contour. `epsilonPixels` is the
// maximum perpendicular deviation a removed point is allowed to have
// introduced; larger values simplify more aggressively. Always keeps at
// least 3 points (a degenerate input shorter than that is returned
// unchanged).
std::vector<Vec2> simplifyClosedPolygon(const std::vector<Vec2>& points, double epsilonPixels);

} // namespace gmcore
