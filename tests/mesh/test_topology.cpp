#include <doctest/doctest.h>

#include <algorithm>
#include <map>
#include <set>
#include <stdexcept>
#include <utility>

#include "hp/mesh/generators.hpp"
#include "hp/mesh/topology.hpp"

namespace {

// Checks that the surface is a closed, consistently wound, edge-manifold sphere-like shell:
// every directed edge appears once and its reverse once, and V - E + F = 2.
void check_closed_surface(const hp::TetMesh& mesh, const hp::Topology& topo) {
    std::map<std::pair<hp::Index, hp::Index>, int> directed;
    std::set<hp::Index> verts;
    for (const hp::Triangle& t : topo.surface_triangles) {
        for (size_t a = 0; a < 3; ++a) {
            ++directed[{t[a], t[(a + 1) % 3]}];
            verts.insert(t[a]);
        }
    }
    for (const auto& [e, count] : directed) {
        REQUIRE(count == 1);
        REQUIRE(directed.count({e.second, e.first}) == 1);
    }
    const auto v = static_cast<long>(verts.size());
    const auto e = static_cast<long>(directed.size() / 2);
    const auto f = static_cast<long>(topo.surface_triangles.size());
    CHECK(v - e + f == 2);

    // Divergence theorem: with outward winding the surface encloses the tet volume.
    double enclosed = 0.0;
    for (const hp::Triangle& t : topo.surface_triangles) {
        const hp::Vec3& a = mesh.rest_positions[static_cast<size_t>(t[0])];
        const hp::Vec3& b = mesh.rest_positions[static_cast<size_t>(t[1])];
        const hp::Vec3& c = mesh.rest_positions[static_cast<size_t>(t[2])];
        enclosed += static_cast<double>(a.dot(b.cross(c))) / 6.0;
    }
    CHECK(enclosed == doctest::Approx(hp::total_volume(mesh)).epsilon(1e-4));
}

// Neighbour lists match the edge list in both directions and are sorted.
void check_neighbors(const hp::TetMesh& mesh, const hp::Topology& topo) {
    REQUIRE(topo.neighbor_offsets.size() == static_cast<size_t>(mesh.num_vertices()) + 1);
    REQUIRE(topo.neighbors.size() == 2 * topo.edges.size());
    const std::set<hp::Edge> edge_set(topo.edges.begin(), topo.edges.end());
    for (hp::Index v = 0; v < mesh.num_vertices(); ++v) {
        const auto begin = topo.neighbors.begin() + topo.neighbor_offsets[static_cast<size_t>(v)];
        const auto end = topo.neighbors.begin() + topo.neighbor_offsets[static_cast<size_t>(v) + 1];
        REQUIRE(begin != end);
        REQUIRE(std::is_sorted(begin, end));
        for (auto it = begin; it != end; ++it) {
            REQUIRE(edge_set.count({std::min(v, *it), std::max(v, *it)}) == 1);
        }
    }
}

// Euler characteristic of a solid ball: V - E + F - T = 1, with F counting every face once.
void check_solid_euler(const hp::TetMesh& mesh, const hp::Topology& topo) {
    const long t = mesh.num_tets();
    const auto b = static_cast<long>(topo.surface_triangles.size());
    const long f = (4 * t + b) / 2;
    CHECK(mesh.num_vertices() - topo.num_edges() + f - t == 1);
}

void check_all(const hp::TetMesh& mesh) {
    const hp::Topology topo = hp::build_topology(mesh);
    REQUIRE(std::is_sorted(topo.edges.begin(), topo.edges.end()));
    for (const hp::Edge& e : topo.edges) REQUIRE(e[0] < e[1]);
    check_neighbors(mesh, topo);
    check_closed_surface(mesh, topo);
    check_solid_euler(mesh, topo);
}

}  // namespace

TEST_CASE("single cube topology") {
    hp::BoxParams p;
    p.cells_x = p.cells_y = p.cells_z = 1;
    const hp::TetMesh mesh = hp::make_box(p);
    const hp::Topology topo = hp::build_topology(mesh);
    CHECK(topo.num_edges() == 18);  // 12 cube edges + 6 face diagonals
    CHECK(topo.surface_triangles.size() == 12);
    check_all(mesh);
}

TEST_CASE("box, beam and sphere surfaces are closed and manifold") {
    check_all(hp::make_box({}));
    check_all(hp::make_beam({}));
    for (const hp::Index n : {1, 2, 3, 5, 8, 10, 12, 16, 21}) {
        CAPTURE(n);
        hp::SphereParams p;
        p.cells_per_diameter = n;
        check_all(hp::make_sphere(p));
    }
}

TEST_CASE("topology rejects a face shared by three tets") {
    hp::TetMesh mesh;
    mesh.rest_positions = {hp::Vec3(0, 0, 0), hp::Vec3(1, 0, 0), hp::Vec3(0, 1, 0),
                           hp::Vec3(0, 0, 1), hp::Vec3(0, 0, -1), hp::Vec3(1, 1, 1)};
    mesh.tets = {{0, 1, 2, 3}, {0, 2, 1, 4}, {0, 1, 2, 5}};
    CHECK_THROWS_AS(hp::build_topology(mesh), std::invalid_argument);
}
