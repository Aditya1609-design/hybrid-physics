#include "core/collider_shapes.hpp"

#include <cmath>

namespace hp::viewer {

// Crosses n with the world axis least aligned with it, so the result is never tiny.
PlaneBasis plane_basis(const Vec3& n) {
    const Vec3 helper = std::abs(n.x()) < 0.9f ? Vec3(Vec3::UnitX()) : Vec3(Vec3::UnitY());
    const Vec3 u = helper.cross(n).normalized();
    return {u, n.cross(u)};
}

std::array<Vec3, 8> box_corners(const Collider& box) {
    std::array<Vec3, 8> corners;
    for (int i = 0; i < 8; ++i) {
        const Vec3 local((i & 1) ? box.half_extents.x() : -box.half_extents.x(),
                         (i & 2) ? box.half_extents.y() : -box.half_extents.y(),
                         (i & 4) ? box.half_extents.z() : -box.half_extents.z());
        corners[static_cast<size_t>(i)] = box.center + box.rotation * local;
    }
    return corners;
}

// Each face lists its four corners in counter-clockwise order around its outward normal.
const std::array<std::array<int, 4>, 6>& box_faces() {
    static const std::array<std::array<int, 4>, 6> faces = {{
        {1, 3, 7, 5},  // +x
        {0, 4, 6, 2},  // -x
        {2, 6, 7, 3},  // +y
        {0, 1, 5, 4},  // -y
        {4, 5, 7, 6},  // +z
        {0, 2, 3, 1},  // -z
    }};
    return faces;
}

// Corners that differ in exactly one bit share an edge.
const std::array<std::array<int, 2>, 12>& box_edges() {
    static const std::array<std::array<int, 2>, 12> edges = {{
        {0, 1}, {2, 3}, {4, 5}, {6, 7},
        {0, 2}, {1, 3}, {4, 6}, {5, 7},
        {0, 4}, {1, 5}, {2, 6}, {3, 7},
    }};
    return edges;
}

}  // namespace hp::viewer
