#include <doctest/doctest.h>

#include <memory>
#include <stdexcept>

#include "hp/mesh/generators.hpp"
#include "hp/scene/make_state.hpp"
#include "hp/solver/solver.hpp"

namespace {

// Moves every owned vertex by velocity * dt; enough to check the interface is usable.
class DriftSolver final : public hp::ISolver {
public:
    void step(hp::SimState& state, const hp::RegionMask& mask, hp::Real dt) override {
        for (hp::Index v = 0; v < state.num_vertices(); ++v) {
            const auto vs = static_cast<size_t>(v);
            if (mask.owns(v)) state.positions[vs] += dt * state.velocities[vs];
        }
    }
};

}  // namespace

TEST_CASE("RegionMask::all owns every vertex") {
    const hp::RegionMask mask = hp::RegionMask::all(7);
    CHECK(mask.num_vertices() == 7);
    CHECK(mask.num_owned() == 7);
    CHECK(mask.covers_all());
    for (hp::Index v = 0; v < 7; ++v) CHECK(mask.owns(v));

    CHECK(hp::RegionMask::all(0).covers_all());
    CHECK_THROWS_AS(hp::RegionMask::all(-1), std::invalid_argument);
}

TEST_CASE("a solver can be stepped through the ISolver interface") {
    hp::SimState s = hp::make_state(hp::make_box({}), 1.0f);
    s.velocities.assign(s.velocities.size(), hp::Vec3(1.0f, 0.0f, 0.0f));
    const hp::Vec3 before = s.positions[3];

    const std::unique_ptr<hp::ISolver> solver = std::make_unique<DriftSolver>();
    solver->step(s, hp::RegionMask::all(s.num_vertices()), 0.5f);
    CHECK(s.positions[3].x() == doctest::Approx(before.x() + 0.5f));
}
