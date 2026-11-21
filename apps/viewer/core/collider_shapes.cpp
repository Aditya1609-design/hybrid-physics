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

std::vector<Vec3> unit_sphere_triangles(int rings, int slices) {
    std::vector<Vec3> tris;
    if (rings < 2 || slices < 3) return tris;
    constexpr float kPi = 3.14159265358979f;
    // Ring i sits at polar angle pi*i/rings from +y; slice j at azimuth 2pi*j/slices.
    auto point = [&](int i, int j) {
        const float theta = kPi * static_cast<float>(i) / static_cast<float>(rings);
        const float phi = 2.0f * kPi * static_cast<float>(j) / static_cast<float>(slices);
        return Vec3(std::sin(theta) * std::cos(phi), std::cos(theta),
                    std::sin(theta) * std::sin(phi));
    };
    auto add = [&](const Vec3& a, const Vec3& b, const Vec3& c) {
        if ((b - a).cross(c - a).norm() > 1e-7f) tris.insert(tris.end(), {a, b, c});
    };
    tris.reserve(static_cast<size_t>(6 * rings * slices));
    for (int i = 0; i < rings; ++i) {
        for (int j = 0; j < slices; ++j) {
            const Vec3 p00 = point(i, j);
            const Vec3 p10 = point(i + 1, j);
            const Vec3 p11 = point(i + 1, j + 1);
            const Vec3 p01 = point(i, j + 1);
            add(p00, p11, p10);
            add(p00, p01, p11);
        }
    }
    return tris;
}

}  // namespace hp::viewer
