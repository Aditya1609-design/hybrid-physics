#include "core/surface_mesh.hpp"

#include <algorithm>
#include <utility>

namespace hp::viewer {

void build_surface(const std::vector<Vec3>& positions, const std::vector<Triangle>& triangles,
                   SurfaceMesh& out) {
    const size_t n = triangles.size();
    out.corners.resize(3 * n);
    out.normals.resize(n);
    out.source.resize(3 * n);
    for (size_t t = 0; t < n; ++t) {
        const Triangle& tri = triangles[t];
        const Vec3& a = positions[static_cast<size_t>(tri[0])];
        const Vec3& b = positions[static_cast<size_t>(tri[1])];
        const Vec3& c = positions[static_cast<size_t>(tri[2])];
        // Topology winds surface triangles so (b-a) x (c-a) points outwards.
        const Vec3 cross = (b - a).cross(c - a);
        const float len = cross.norm();
        out.normals[t] = len > 0.0f ? Vec3(cross / len) : Vec3(Vec3::UnitY());
        for (size_t k = 0; k < 3; ++k) {
            out.corners[3 * t + k] = positions[static_cast<size_t>(tri[k])];
            out.source[3 * t + k] = tri[k];
        }
    }
}

std::vector<Edge> surface_edges(const std::vector<Triangle>& triangles) {
    std::vector<Edge> edges;
    edges.reserve(3 * triangles.size());
    for (const Triangle& tri : triangles) {
        for (size_t k = 0; k < 3; ++k) {
            Index a = tri[k];
            Index b = tri[(k + 1) % 3];
            if (a > b) std::swap(a, b);
            edges.push_back({a, b});
        }
    }
    std::sort(edges.begin(), edges.end());
    edges.erase(std::unique(edges.begin(), edges.end()), edges.end());
    return edges;
}

}  // namespace hp::viewer
