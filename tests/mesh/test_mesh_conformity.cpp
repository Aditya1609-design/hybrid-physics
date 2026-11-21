#include <doctest/doctest.h>

#include <algorithm>
#include <cmath>
#include <map>
#include <vector>

#include "hp/mesh/generators.hpp"
#include "hp/mesh/topology.hpp"

namespace {

// Every surface vertex must have a disc-shaped neighbourhood: the triangles around it,
// linked through shared edges, form exactly one closed loop. Two voxels touching only at a
// corner or along an edge would give a vertex with two loops (or an edge with four faces).
void check_vertex_manifold(const hp::Topology& topo) {
    // For each surface vertex, its triangles as directed "next -> prev" links around it.
    std::map<hp::Index, std::map<hp::Index, hp::Index>> link;
    std::map<hp::Index, int> link_size;
    for (const hp::Triangle& t : topo.surface_triangles) {
        for (size_t a = 0; a < 3; ++a) {
            const hp::Index v = t[a], next = t[(a + 1) % 3], prev = t[(a + 2) % 3];
            auto& m = link[v];
            REQUIRE(m.count(next) == 0);  // a repeated link edge means a non-manifold edge
            m[next] = prev;
            ++link_size[v];
        }
    }
    for (const auto& entry : link) {
        const hp::Index v = entry.first;
        const auto& m = entry.second;
        CAPTURE(v);
        // Walk the loop from any start; it must visit every triangle around v once.
        const hp::Index start = m.begin()->first;
        hp::Index cur = start;
        int steps = 0;
        do {
            const auto it = m.find(cur);
            REQUIRE(it != m.end());
            cur = it->second;
            ++steps;
        } while (cur != start && steps <= link_size[v]);
        CHECK(cur == start);
        CHECK(steps == link_size[v]);
    }
}

// Interior faces are shared by exactly two tets; boundary faces lie on the box's outer faces.
void check_box_faces(const hp::TetMesh& mesh, const hp::Topology& topo, const hp::Vec3& lo,
                     const hp::Vec3& hi) {
    const float tol = 1e-5f * (hi - lo).maxCoeff();
    for (const hp::Triangle& t : topo.surface_triangles) {
        bool on_side = false;
        for (int axis = 0; axis < 3 && !on_side; ++axis) {
            for (const float plane : {lo[axis], hi[axis]}) {
                bool all = true;
                for (const hp::Index v : t) {
                    all = all && std::abs(mesh.rest_positions[static_cast<size_t>(v)][axis] -
                                          plane) <= tol;
                }
                on_side = on_side || all;
            }
        }
        CHECK(on_side);
    }
    // Total surface area equals the box's area (no holes, no internal boundary).
    double area = 0.0;
    for (const hp::Triangle& t : topo.surface_triangles) {
        const hp::Vec3& a = mesh.rest_positions[static_cast<size_t>(t[0])];
        const hp::Vec3& b = mesh.rest_positions[static_cast<size_t>(t[1])];
        const hp::Vec3& c = mesh.rest_positions[static_cast<size_t>(t[2])];
        area += 0.5 * static_cast<double>((b - a).cross(c - a).norm());
    }
    const hp::Vec3 s = hi - lo;
    const double expected = 2.0 * (static_cast<double>(s.x()) * s.y() +
                                   static_cast<double>(s.y()) * s.z() +
                                   static_cast<double>(s.z()) * s.x());
    CHECK(area == doctest::Approx(expected).epsilon(1e-5));
}

}  // namespace

TEST_CASE("sphere surfaces are vertex-manifold at every resolution up to 40 cells") {
    for (hp::Index n = 1; n <= 40; ++n) {
        CAPTURE(n);
        hp::SphereParams p;
        p.cells_per_diameter = n;
        p.radius = 0.37f;
        const hp::TetMesh mesh = hp::make_sphere(p);
        REQUIRE_NOTHROW(mesh.validate());
        check_vertex_manifold(hp::build_topology(mesh));
    }
}

TEST_CASE("box and beam surfaces are manifold and cover exactly the box faces") {
    struct Case {
        hp::Vec3 size;
        hp::Index nx, ny, nz;
    };
    const Case cases[] = {{hp::Vec3(1, 1, 1), 1, 1, 1},        {hp::Vec3(1, 1, 1), 2, 3, 4},
                          {hp::Vec3(2, 0.5f, 0.5f), 7, 1, 2},  {hp::Vec3(0.3f, 1.7f, 0.9f), 5, 9, 4},
                          {hp::Vec3(1, 1, 1), 8, 8, 8},        {hp::Vec3(4, 0.2f, 0.2f), 40, 2, 2}};
    for (const Case& c : cases) {
        CAPTURE(c.nx);
        CAPTURE(c.ny);
        CAPTURE(c.nz);
        hp::BoxParams p;
        p.size = c.size;
        p.cells_x = c.nx;
        p.cells_y = c.ny;
        p.cells_z = c.nz;
        p.center = hp::Vec3(0.1f, -0.2f, 0.3f);
        const hp::TetMesh mesh = hp::make_box(p);
        const hp::Topology topo = hp::build_topology(mesh);
        check_vertex_manifold(topo);
        check_box_faces(mesh, topo, p.center - p.size / 2, p.center + p.size / 2);
    }
    const hp::TetMesh beam = hp::make_beam({});
    check_vertex_manifold(hp::build_topology(beam));
}
