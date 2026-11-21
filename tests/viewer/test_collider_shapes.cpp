#include <doctest/doctest.h>

#include <Eigen/Geometry>
#include <cmath>
#include <set>

#include "core/collider_shapes.hpp"

using hp::Collider;
using hp::Vec3;
using namespace hp::viewer;

TEST_CASE("plane basis is orthonormal and right-handed for any normal") {
    for (const Vec3& raw : {Vec3(0, 1, 0), Vec3(1, 0, 0), Vec3(0, 0, -1), Vec3(-1, 0, 0),
                            Vec3(0.4226f, 0.9063f, 0), Vec3(1, 2, -3), Vec3(-0.95f, 0.1f, 0.3f)}) {
        const Vec3 n = raw.normalized();
        const PlaneBasis b = plane_basis(n);
        CAPTURE(n.transpose());
        CHECK(b.u.norm() == doctest::Approx(1.0f).epsilon(1e-5));
        CHECK(b.v.norm() == doctest::Approx(1.0f).epsilon(1e-5));
        CHECK(std::abs(b.u.dot(n)) < 1e-5f);
        CHECK(std::abs(b.v.dot(n)) < 1e-5f);
        CHECK(std::abs(b.u.dot(b.v)) < 1e-5f);
        CHECK((b.u.cross(b.v) - n).norm() < 1e-5f);
    }
}

TEST_CASE("box corners, faces and edges match the collider") {
    const hp::Mat3 rot =
        Eigen::AngleAxisf(0.7f, Vec3(1, 2, 3).normalized()).toRotationMatrix();
    const Collider box = Collider::box(Vec3(0.5f, 0.3f, -1.0f), Vec3(0.15f, 0.3f, 0.6f), rot, 0.5f);
    const auto corners = box_corners(box);

    // Every corner lies on the collider's surface.
    for (const Vec3& c : corners) {
        Vec3 n;
        CHECK(std::abs(hp::signed_distance(box, c, n)) < 1e-5f);
    }

    // Faces are planar quads wound counter-clockwise around their outward normal.
    std::set<int> used;
    for (const auto& f : box_faces()) {
        const Vec3& a = corners[static_cast<size_t>(f[0])];
        const Vec3& b = corners[static_cast<size_t>(f[1])];
        const Vec3& c = corners[static_cast<size_t>(f[2])];
        const Vec3& d = corners[static_cast<size_t>(f[3])];
        const Vec3 normal = (b - a).cross(c - a).normalized();
        const Vec3 centre = 0.25f * (a + b + c + d);
        CHECK(normal.dot(centre - box.center) > 0.0f);
        CHECK(std::abs(normal.dot(d - a)) < 1e-5f);
        // The second triangle of the quad has the same orientation.
        CHECK((c - a).cross(d - a).normalized().dot(normal) > 0.99f);
        for (int i : f) used.insert(i);
    }
    CHECK(used.size() == 8);

    // Each edge runs along one box axis with the full extent on that axis.
    std::set<std::pair<int, int>> distinct;
    for (const auto& e : box_edges()) {
        distinct.insert({e[0], e[1]});
        const int axis_bit = e[0] ^ e[1];
        REQUIRE((axis_bit == 1 || axis_bit == 2 || axis_bit == 4));
        const int axis = axis_bit == 1 ? 0 : (axis_bit == 2 ? 1 : 2);
        const Vec3 d = corners[static_cast<size_t>(e[1])] - corners[static_cast<size_t>(e[0])];
        const float len = d.norm();
        CHECK(len == doctest::Approx(2.0f * box.half_extents[axis]).epsilon(1e-5));
    }
    CHECK(distinct.size() == 12);
}

TEST_CASE("unit sphere triangles lie on the sphere, face outwards and close the surface") {
    const int rings = 12;
    const int slices = 16;
    const std::vector<Vec3> tris = unit_sphere_triangles(rings, slices);
    REQUIRE(tris.size() % 3 == 0);
    // Two triangles per quad, minus one degenerate triangle per slice at each pole.
    CHECK(tris.size() / 3 == static_cast<size_t>(2 * rings * slices - 2 * slices));

    double area = 0.0;
    for (size_t t = 0; t < tris.size(); t += 3) {
        const Vec3 cross = (tris[t + 1] - tris[t]).cross(tris[t + 2] - tris[t]);
        const Vec3 centroid = (tris[t] + tris[t + 1] + tris[t + 2]) / 3.0f;
        CHECK(cross.dot(centroid) > 0.0f);
        area += 0.5 * static_cast<double>(cross.norm());
        for (size_t k = 0; k < 3; ++k) {
            CHECK(tris[t + k].norm() == doctest::Approx(1.0f).epsilon(1e-5));
        }
    }
    // An inscribed polyhedron is slightly smaller than the sphere (4 pi = 12.566).
    CHECK(area < 4.0 * 3.14159265358979);
    CHECK(area > 0.97 * 4.0 * 3.14159265358979);

    CHECK(unit_sphere_triangles(1, 16).empty());
    CHECK(unit_sphere_triangles(8, 2).empty());
}
