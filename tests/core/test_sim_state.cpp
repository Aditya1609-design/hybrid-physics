#include <doctest/doctest.h>

#include <limits>
#include <stdexcept>

#include "hp/core/sim_state.hpp"

namespace {

// Two bodies of 2 and 3 vertices, all defaults.
hp::SimState two_body_state() {
    hp::SimState s;
    const size_t n = 5;
    s.positions.assign(n, hp::Vec3::Zero());
    s.velocities.assign(n, hp::Vec3::Zero());
    s.rest_positions.assign(n, hp::Vec3::Zero());
    s.inv_mass.assign(n, 1.0f);
    s.region_id.assign(n, 0);
    s.solver_owner.assign(n, hp::SolverId::Xpbd);
    s.body_id = {0, 0, 1, 1, 1};
    s.bodies = {{0, 2}, {2, 3}};
    return s;
}

}  // namespace

TEST_CASE("simstate accepts a consistent multi-body state") {
    CHECK_NOTHROW(two_body_state().validate());
}

TEST_CASE("simstate rejects inconsistent states") {
    hp::SimState s = two_body_state();
    s.region_id.pop_back();
    CHECK_THROWS_AS(s.validate(), std::invalid_argument);

    s = two_body_state();
    s.body_id[1] = 1;
    CHECK_THROWS_AS(s.validate(), std::invalid_argument);

    s = two_body_state();
    s.bodies[1].first_vertex = 3;
    CHECK_THROWS_AS(s.validate(), std::invalid_argument);

    s = two_body_state();
    s.bodies.pop_back();
    CHECK_THROWS_AS(s.validate(), std::invalid_argument);

    s = two_body_state();
    s.inv_mass[4] = -1.0f;
    CHECK_THROWS_AS(s.validate(), std::invalid_argument);

    s = two_body_state();
    s.inv_mass[0] = std::numeric_limits<float>::infinity();
    CHECK_THROWS_AS(s.validate(), std::invalid_argument);

    s = two_body_state();
    s.inv_mass[0] = 0.0f;  // pinned vertices are allowed
    CHECK_NOTHROW(s.validate());
}
