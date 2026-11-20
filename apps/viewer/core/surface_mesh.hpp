#pragma once

#include <vector>

#include "hp/core/types.hpp"
#include "hp/mesh/topology.hpp"

namespace hp::viewer {

// Surface triangles unrolled for flat shading: three corners per triangle, one normal per
// triangle, and the body vertex each corner came from (to look up per-vertex colors).
struct SurfaceMesh {
    std::vector<Vec3> corners;   // 3 per triangle
    std::vector<Vec3> normals;   // 1 per triangle, unit, pointing out of the body
    std::vector<Index> source;   // 3 per triangle: vertex index into the SimState arrays

    Index num_triangles() const { return static_cast<Index>(normals.size()); }
};

// Fills `out` from the current positions. Reuses its buffers, so calling it every frame does
// not allocate after the first time. A degenerate triangle gets the normal +y.
void build_surface(const std::vector<Vec3>& positions, const std::vector<Triangle>& triangles,
                   SurfaceMesh& out);

// Unique edges of the surface triangles, each with edge[0] < edge[1], sorted.
std::vector<Edge> surface_edges(const std::vector<Triangle>& triangles);

}  // namespace hp::viewer
