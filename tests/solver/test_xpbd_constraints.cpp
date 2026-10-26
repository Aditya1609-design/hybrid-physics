#include <doctest/doctest.h>

#include <limits>
#include <stdexcept>

#include "hp/mesh/generators.hpp"
#include "hp/solver/xpbd_constraints.hpp"

TEST_CASE("constraints match the rest shape") {
    const hp::TetMesh mesh = hp::make_sphere({});
    const hp::Topology topo = hp::build_topology(mesh);
    const hp::XpbdConstraints c = hp::build_constraints(mesh, topo, {});

    REQUIRE(c.num_edges() == topo.num_edges());
    REQUIRE(c.num_tets() == mesh.num_tets());
    REQUIRE(c.edge_rest_length.size() == c.edges.size());
    REQUIRE(c.edge_compliance.size() == c.edges.size());
    REQUIRE(c.tet_rest_volume.size() == c.tets.size());
    REQUIRE(c.tet_compliance.size() == c.tets.size());

    // At rest every constraint function is zero.
    for (size_t e = 0; e < c.edges.size(); ++e) {
        const hp::Vec3 d = mesh.rest_positions[static_cast<size_t>(c.edges[e][0])] -
                           mesh.rest_positions[static_cast<size_t>(c.edges[e][1])];
        CHECK(d.norm() - c.edge_rest_length[e] == 0.0f);
        CHECK(c.edge_compliance[e] > 0.0f);
    }
    double volume = 0.0;
    for (size_t t = 0; t < c.tets.size(); ++t) {
        CHECK(c.tet_rest_volume[t] == hp::tet_volume(mesh, static_cast<hp::Index>(t)));
        volume += c.tet_rest_volume[t];
    }
    CHECK(volume == doctest::Approx(hp::total_volume(mesh)).epsilon(1e-6));
}

TEST_CASE("edge volume shares add up to the mesh volume") {
    // alpha_e = c * L^2 / V_e, so sum of c * L^2 / alpha_e recovers the total volume.
    hp::Material material;
    material.edge_compliance = 2e-6f;
    material.volume_compliance = 3e-7f;
    const hp::TetMesh mesh = hp::make_beam({});
    const hp::XpbdConstraints c = hp::build_constraints(mesh, hp::build_topology(mesh), material);

    double edge_volume = 0.0;
    for (size_t e = 0; e < c.edges.size(); ++e) {
        const double l = c.edge_rest_length[e];
        edge_volume += 2e-6 * l * l / static_cast<double>(c.edge_compliance[e]);
    }
    CHECK(edge_volume == doctest::Approx(hp::total_volume(mesh)).epsilon(1e-5));
    for (size_t t = 0; t < c.tets.size(); ++t) {
        CHECK(c.tet_compliance[t] == doctest::Approx(3e-7f * c.tet_rest_volume[t]));
    }
}

TEST_CASE("zero compliance gives perfectly stiff constraints") {
    hp::Material material;
    material.edge_compliance = 0.0f;
    material.volume_compliance = 0.0f;
    const hp::TetMesh mesh = hp::make_box({});
    const hp::XpbdConstraints c = hp::build_constraints(mesh, hp::build_topology(mesh), material);
    for (const float a : c.edge_compliance) CHECK(a == 0.0f);
    for (const float a : c.tet_compliance) CHECK(a == 0.0f);
}

TEST_CASE("invalid materials are rejected") {
    const hp::TetMesh mesh = hp::make_box({});
    const hp::Topology topo = hp::build_topology(mesh);
    hp::Material m;
    m.density = 0.0f;
    CHECK_THROWS_AS(hp::build_constraints(mesh, topo, m), std::invalid_argument);
    m = {};
    m.edge_compliance = -1.0f;
    CHECK_THROWS_AS(hp::build_constraints(mesh, topo, m), std::invalid_argument);
    m = {};
    m.damping = std::numeric_limits<float>::infinity();
    CHECK_THROWS_AS(m.validate(), std::invalid_argument);
}
