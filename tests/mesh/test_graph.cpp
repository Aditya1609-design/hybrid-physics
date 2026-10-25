#include <doctest/doctest.h>

#include <cstring>
#include <vector>

#include "hp/mesh/generators.hpp"
#include "hp/mesh/graph.hpp"
#include "hp/scene/make_state.hpp"

TEST_CASE("graph has paired directed edges with consistent rest geometry") {
    const hp::TetMesh mesh = hp::make_sphere({});
    const hp::Topology topo = hp::build_topology(mesh);
    const hp::MeshGraph g = hp::build_graph(mesh, topo);

    const hp::Index e = topo.num_edges();
    REQUIRE(g.num_nodes == mesh.num_vertices());
    REQUIRE(g.num_directed_edges() == 2 * e);
    REQUIRE(g.receivers.size() == g.senders.size());
    REQUIRE(g.rest_lengths.size() == g.senders.size());
    REQUIRE(g.rel_rest_pos.size() == g.senders.size());

    std::vector<int> in_degree(static_cast<size_t>(g.num_nodes), 0);
    for (hp::Index k = 0; k < 2 * e; ++k) {
        const auto ks = static_cast<size_t>(k);
        const auto rs = static_cast<size_t>((k + e) % (2 * e));  // the reverse edge
        const hp::Index s = g.senders[ks], r = g.receivers[ks];
        REQUIRE(s >= 0);
        REQUIRE(r >= 0);
        REQUIRE(s < g.num_nodes);
        REQUIRE(r < g.num_nodes);
        REQUIRE(s != r);
        CHECK(g.senders[rs] == r);
        CHECK(g.receivers[rs] == s);
        CHECK(g.rel_rest_pos[rs] == -g.rel_rest_pos[ks]);

        const hp::Vec3 d = mesh.rest_positions[static_cast<size_t>(s)] -
                           mesh.rest_positions[static_cast<size_t>(r)];
        CHECK(g.rel_rest_pos[ks] == d);
        CHECK(g.rest_lengths[ks] == doctest::Approx(d.norm()));
        CHECK(g.rest_lengths[ks] > 0.0f);
        ++in_degree[static_cast<size_t>(r)];
    }
    for (const int d : in_degree) CHECK(d > 0);
}

TEST_CASE("graph arrays are flat float32 and int32 blocks") {
    const hp::TetMesh mesh = hp::make_box({});
    const hp::MeshGraph g = hp::build_graph(mesh, hp::build_topology(mesh));
    // rel_rest_pos reads as a row-major [2E, 3] float array with no padding.
    const float* flat = g.rel_rest_pos.data()->data();
    CHECK(flat[3 * 5 + 1] == g.rel_rest_pos[5].y());
    CHECK(sizeof(g.senders[0]) == 4);
}

TEST_CASE("graph export is deterministic") {
    const hp::TetMesh mesh = hp::make_beam({});
    const hp::MeshGraph a = hp::build_graph(mesh, hp::build_topology(mesh));
    const hp::TetMesh again = hp::make_beam({});
    const hp::MeshGraph b = hp::build_graph(again, hp::build_topology(again));
    CHECK(a.senders == b.senders);
    CHECK(a.receivers == b.receivers);
    CHECK(a.rest_lengths == b.rest_lengths);
    CHECK(std::memcmp(a.rel_rest_pos.data(), b.rel_rest_pos.data(),
                      a.rel_rest_pos.size() * sizeof(hp::Vec3)) == 0);
}

TEST_CASE("node features hold velocity and a one-hot node type") {
    hp::BoxParams p;
    p.cells_x = p.cells_y = p.cells_z = 2;
    hp::SimState s = hp::make_state(hp::make_box(p), 1.0f);
    s.velocities[4] = hp::Vec3(1.0f, -2.0f, 3.0f);
    s.inv_mass[7] = 0.0f;

    const std::vector<float> f = hp::node_features(s);
    const auto dim = static_cast<size_t>(hp::kNodeFeatureDim);
    REQUIRE(f.size() == static_cast<size_t>(s.num_vertices()) * dim);
    CHECK(f[4 * dim + 0] == 1.0f);
    CHECK(f[4 * dim + 1] == -2.0f);
    CHECK(f[4 * dim + 2] == 3.0f);
    CHECK(f[4 * dim + 3] == 1.0f);  // free
    CHECK(f[4 * dim + 4] == 0.0f);
    CHECK(f[7 * dim + 3] == 0.0f);
    CHECK(f[7 * dim + 4] == 1.0f);  // pinned
}
