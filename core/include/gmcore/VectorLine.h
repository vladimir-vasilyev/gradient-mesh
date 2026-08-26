// VectorLine.h — a user-drawn guide polyline (Sun et al. 2007, Sec. 4.3):
// an optional, soft directional constraint nudging nearby mesh edges to
// align with an artist-specified direction (e.g. along a highlight or a
// fold), instead of being purely dictated by the reconstruction error.
#pragma once
#include "gmcore/Vec2.h"
#include <vector>

namespace gmcore {

struct VectorLine {
    std::vector<Vec2> points; // polyline in image pixel coordinates, >= 2 points
};

} // namespace gmcore
