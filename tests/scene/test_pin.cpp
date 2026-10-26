#include <doctest/doctest.h>

#include "hp/mesh/generators.hpp"
#include "hp/scene/make_state.hpp"
#include "hp/scene/pin.hpp"

TEST_CASE("pin_vertices pins exactly the selected vertices") {
    const hp::BeamParams params;
    const hp::TetMesh mesh = hp::make_beam(params);
    hp::SimState s = hp::make_state(mesh, 1000.0f);
    s.velocities.assign(s.velocities.size(), hp::Vec3(1.0f, 2.0f, 3.0f));

    // The x = min end face of the beam has (cells_across + 1)^2 vertices.
    const float x_min = -0.5f * params.length;
    const hp::Index pinned =
        hp::pin_vertices(s, [&](const hp::Vec3& p) { return p.x() < x_min + 1e-4f; });
    CHECK(pinned == (params.cells_across + 1) * (params.cells_across + 1));

    hp::Index zero_mass = 0;
    for (size_t v = 0; v < s.positions.size(); ++v) {
        const bool selected = s.rest_positions[v].x() < x_min + 1e-4f;
        if (selected) {
            CHECK(s.inv_mass[v] == 0.0f);
            CHECK(s.velocities[v] == hp::Vec3::Zero());
            ++zero_mass;
        } else {
            CHECK(s.inv_mass[v] > 0.0f);
            CHECK(s.velocities[v] == hp::Vec3(1.0f, 2.0f, 3.0f));
        }
    }
    CHECK(zero_mass == pinned);
    CHECK_NOTHROW(s.validate());
}
