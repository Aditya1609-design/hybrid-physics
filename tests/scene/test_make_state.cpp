#include <doctest/doctest.h>

#include <stdexcept>
#include <utility>

#include "hp/mesh/generators.hpp"
#include "hp/mesh/mass.hpp"
#include "hp/scene/make_state.hpp"

TEST_CASE("lumped masses sum to density times volume") {
    const hp::TetMesh mesh = hp::make_beam({});
    const std::vector<float> mass = hp::lumped_masses(mesh, 1000.0f);
    double sum = 0.0;
    for (const float m : mass) {
        REQUIRE(m > 0.0f);
        sum += m;
    }
    CHECK(sum == doctest::Approx(1000.0 * hp::total_volume(mesh)).epsilon(1e-5));
    CHECK_THROWS_AS(hp::lumped_masses(mesh, 0.0f), std::invalid_argument);
}

TEST_CASE("make_state builds a valid single-body state at rest") {
    const hp::TetMesh mesh = hp::make_sphere({});
    const hp::SimState s = hp::make_state(mesh, 500.0f);
    CHECK_NOTHROW(s.validate());
    REQUIRE(s.num_vertices() == mesh.num_vertices());
    REQUIRE(s.bodies.size() == 1);
    CHECK(s.bodies[0].num_vertices == mesh.num_vertices());

    const std::vector<float> mass = hp::lumped_masses(mesh, 500.0f);
    for (size_t v = 0; v < s.positions.size(); ++v) {
        CHECK(s.positions[v] == mesh.rest_positions[v]);
        CHECK(s.rest_positions[v] == mesh.rest_positions[v]);
        CHECK(s.velocities[v] == hp::Vec3::Zero());
        CHECK(s.inv_mass[v] * mass[v] == doctest::Approx(1.0f));
        CHECK(s.region_id[v] == 0);
        CHECK(s.solver_owner[v] == hp::SolverId::Xpbd);
        CHECK(s.body_id[v] == 0);
    }
}

TEST_CASE("make_state rejects an invalid mesh") {
    hp::TetMesh mesh = hp::make_box({});
    std::swap(mesh.tets[0][0], mesh.tets[0][1]);
    CHECK_THROWS_AS(hp::make_state(mesh, 1.0f), std::invalid_argument);
}
