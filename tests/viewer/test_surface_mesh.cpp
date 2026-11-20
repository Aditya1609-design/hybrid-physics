#include <doctest/doctest.h>

#include <algorithm>
#include <cmath>

#include "core/surface_mesh.hpp"
#include "hp/mesh/generators.hpp"
#include "hp/mesh/topology.hpp"

using hp::Vec3;

TEST_CASE("surface mesh has one flat outward normal per surface triangle") {
    hp::BoxParams params;
    params.size = Vec3(1.0f, 0.6f, 0.8f);
    params.cells_x = 3;
    params.cells_y = 2;
    params.cells_z = 3;
    params.center = Vec3(0.5f, 1.0f, -0.2f);
    const hp::TetMesh mesh = hp::make_box(params);
    const hp::Topology topo = hp::build_topology(mesh);

    hp::viewer::SurfaceMesh surf;
    hp::viewer::build_surface(mesh.rest_positions, topo.surface_triangles, surf);
    REQUIRE(surf.num_triangles() == static_cast<hp::Index>(topo.surface_triangles.size()));
    REQUIRE(surf.corners.size() == 3 * topo.surface_triangles.size());
    REQUIRE(surf.source.size() == surf.corners.size());

    for (size_t t = 0; t < surf.normals.size(); ++t) {
        const Vec3& n = surf.normals[t];
        CHECK(n.norm() == doctest::Approx(1.0f).epsilon(1e-5));
        // A box surface is axis-aligned: each normal is one of the six axis directions.
        CHECK(n.cwiseAbs().maxCoeff() == doctest::Approx(1.0f).epsilon(1e-5));
        const Vec3 centroid = (surf.corners[3 * t] + surf.corners[3 * t + 1] +
                               surf.corners[3 * t + 2]) / 3.0f;
        CHECK(n.dot(centroid - params.center) > 0.0f);
        for (size_t k = 0; k < 3; ++k) {
            const auto v = static_cast<size_t>(surf.source[3 * t + k]);
            CHECK(surf.corners[3 * t + k] == mesh.rest_positions[v]);
        }
    }

    // Rebuilding with moved positions reuses the buffers and follows the new positions.
    std::vector<Vec3> moved = mesh.rest_positions;
    for (Vec3& x : moved) x += Vec3(0, 2, 0);
    hp::viewer::build_surface(moved, topo.surface_triangles, surf);
    CHECK(surf.num_triangles() == static_cast<hp::Index>(topo.surface_triangles.size()));
    CHECK(surf.corners[0] == moved[static_cast<size_t>(surf.source[0])]);
}

TEST_CASE("a degenerate triangle gets the +y normal instead of NaN") {
    const std::vector<Vec3> x = {Vec3(0, 0, 0), Vec3(1, 0, 0), Vec3(2, 0, 0)};
    hp::viewer::SurfaceMesh surf;
    hp::viewer::build_surface(x, {{0, 1, 2}}, surf);
    REQUIRE(surf.num_triangles() == 1);
    CHECK(surf.normals[0] == Vec3::UnitY());
}

TEST_CASE("surface edges are unique and each is shared by two triangles of a closed surface") {
    hp::SphereParams params;
    params.radius = 0.5f;
    params.cells_per_diameter = 6;
    const hp::TetMesh mesh = hp::make_sphere(params);
    const hp::Topology topo = hp::build_topology(mesh);
    const std::vector<hp::Edge> edges = hp::viewer::surface_edges(topo.surface_triangles);

    CHECK(std::is_sorted(edges.begin(), edges.end()));
    CHECK(std::adjacent_find(edges.begin(), edges.end()) == edges.end());
    for (const hp::Edge& e : edges) CHECK(e[0] < e[1]);
    // Closed triangle surface: every edge borders exactly two triangles, so E = 3F / 2.
    CHECK(2 * edges.size() == 3 * topo.surface_triangles.size());
}
