#include <doctest/doctest.h>

#include <cstring>
#include <string>
#include <vector>

#include "core/sim_driver.hpp"
#include "hp/metrics/energy.hpp"
#include "hp/scene/scene.hpp"
#include "hp/solver/xpbd.hpp"

using hp::viewer::FixedStepPacer;
using hp::viewer::SimDriver;

namespace {

// 1/64 s is exact in binary, so step counts below do not depend on rounding.
constexpr double kDt = 1.0 / 64.0;

hp::Scenario cube_drop() {
    return hp::load_scenario(std::string(HP_SCENARIO_DIR) + "/cube_drop.json");
}

template <typename T>
bool same_bytes(const std::vector<T>& a, const std::vector<T>& b) {
    return a.size() == b.size() && std::memcmp(a.data(), b.data(), a.size() * sizeof(T)) == 0;
}

bool same_state(const hp::SimState& a, const hp::SimState& b) {
    return same_bytes(a.positions, b.positions) && same_bytes(a.velocities, b.velocities);
}

}  // namespace

TEST_CASE("pacer turns wall time into whole fixed steps and carries the remainder") {
    FixedStepPacer pacer(kDt);
    CHECK(pacer.steps_for(0.0) == 0);
    CHECK(pacer.steps_for(-1.0) == 0);
    CHECK(pacer.steps_for(0.5 * kDt) == 0);
    CHECK(pacer.steps_for(0.5 * kDt) == 1);  // two halves make one step
    CHECK(pacer.steps_for(2.25 * kDt) == 2);
    CHECK(pacer.steps_for(0.75 * kDt) == 1);  // 0.25 left over plus 0.75
    CHECK_FALSE(pacer.behind());

    // 10 s of 60 Hz frames gives 10 s of 1/64 s steps: one per frame, two when the
    // remainder adds up. 600 * (1/60) is not exact in binary, so allow 639 or 640.
    pacer.reset();
    int total = 0;
    for (int i = 0; i < 600; ++i) {
        const int n = pacer.steps_for(1.0 / 60.0);
        CHECK(n >= 1);
        CHECK(n <= 2);
        total += n;
    }
    CHECK(total >= 639);
    CHECK(total <= 640);
    CHECK_FALSE(pacer.behind());
}

TEST_CASE("pacer caps steps per call and drops the backlog when it falls behind") {
    FixedStepPacer pacer(kDt);
    CHECK(pacer.steps_for(10.0 * kDt) == FixedStepPacer::kMaxSteps);
    CHECK(pacer.behind());
    // The backlog is gone: the next short call does not try to catch up.
    CHECK(pacer.steps_for(0.5 * kDt) == 0);
    CHECK_FALSE(pacer.behind());

    // Exactly the cap is not "behind".
    pacer.reset();
    CHECK(pacer.steps_for(4.0 * kDt) == 4);
    CHECK_FALSE(pacer.behind());

    // A very long stall is cut to kMaxWallDt before counting.
    pacer.reset();
    CHECK(pacer.steps_for(1000.0) == FixedStepPacer::kMaxSteps);
    CHECK(pacer.behind());
}

TEST_CASE("sim driver frames are bit-identical to stepping the scene directly") {
    const hp::Scenario scenario = cube_drop();
    SimDriver driver(scenario);
    hp::Scene direct = hp::build_scene(scenario);
    const hp::RegionMask mask = hp::RegionMask::all(direct.state.num_vertices());

    CHECK(same_state(driver.scene().state, direct.state));
    CHECK(driver.state_hash() != 0);
    for (int f = 0; f < 90; ++f) {
        driver.step();
        direct.solver.step(direct.state, mask, hp::kFrameDt);
    }
    CHECK(driver.frame() == 90);
    CHECK(driver.sim_time() == doctest::Approx(90.0 * static_cast<double>(hp::kFrameDt)));
    CHECK(same_state(driver.scene().state, direct.state));
    CHECK_FALSE(driver.failed());
    CHECK(driver.last_step_ms() >= 0.0);

    // Metrics are the engine's own functions on the same state.
    const hp::Energy e = hp::energy(direct.state, direct.solver.constraints(),
                                    direct.solver.settings().gravity);
    const hp::FrameMetrics m = driver.metrics();
    CHECK(m.energy.kinetic == e.kinetic);
    CHECK(m.energy.gravitational == e.gravitational);
    CHECK(m.energy.elastic == e.elastic);
}

TEST_CASE("sim driver advance takes the pacer's steps, and reset restores frame 0 exactly") {
    SimDriver driver(cube_drop());
    const std::uint64_t initial_hash = driver.state_hash();
    const hp::SimState initial = driver.scene().state;
    const double dt = static_cast<double>(hp::kFrameDt);

    int taken = 0;
    for (double wall : {0.5 * dt, 0.6 * dt, 3.2 * dt, 0.0, 1.0 * dt}) taken += driver.advance(wall);
    CHECK(taken == driver.frame());
    CHECK(taken == 5);  // 5.3 frames of wall time
    CHECK(driver.state_hash() != initial_hash);

    CHECK(driver.advance(100.0 * dt) == FixedStepPacer::kMaxSteps);
    CHECK(driver.behind());

    // A frame limit stops exactly on that frame even when the pacer asks for more steps.
    const hp::Index limit = driver.frame() + 2;
    CHECK(driver.advance(4.0 * dt, limit) == 2);
    CHECK(driver.frame() == limit);
    CHECK(driver.advance(4.0 * dt, limit) == 0);
    CHECK(driver.frame() == limit);

    driver.reset();
    CHECK(driver.frame() == 0);
    CHECK_FALSE(driver.behind());
    CHECK(driver.state_hash() == initial_hash);
    CHECK(same_state(driver.scene().state, initial));
}
