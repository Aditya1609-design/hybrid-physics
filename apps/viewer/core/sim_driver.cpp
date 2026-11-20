#include "core/sim_driver.hpp"

#include <algorithm>
#include <chrono>
#include <utility>
#include <vector>

#include "hp/solver/xpbd.hpp"

namespace hp::viewer {

int FixedStepPacer::steps_for(double wall_dt) {
    accumulator_ += std::clamp(wall_dt, 0.0, kMaxWallDt);
    int steps = 0;
    while (accumulator_ >= dt_ && steps < kMaxSteps) {
        accumulator_ -= dt_;
        ++steps;
    }
    // Still a full step owed after the cap: drop the backlog rather than chase it.
    behind_ = accumulator_ >= dt_;
    if (behind_) accumulator_ = 0.0;
    return steps;
}

void FixedStepPacer::reset() {
    accumulator_ = 0.0;
    behind_ = false;
}

namespace {

template <typename T>
void fnv1a(std::uint64_t& h, const std::vector<T>& data) {
    const auto* bytes = reinterpret_cast<const unsigned char*>(data.data());
    for (size_t i = 0; i < data.size() * sizeof(T); ++i) {
        h ^= bytes[i];
        h *= 0x100000001b3ULL;
    }
}

bool all_finite(const SimState& s) {
    for (size_t v = 0; v < s.positions.size(); ++v) {
        if (!s.positions[v].allFinite() || !s.velocities[v].allFinite()) return false;
    }
    return true;
}

}  // namespace

SimDriver::SimDriver(Scenario scenario)
    : scenario_(std::move(scenario)), pacer_(static_cast<double>(kFrameDt)) {
    reset();
}

void SimDriver::reset() {
    scene_.emplace(build_scene(scenario_));
    mask_ = RegionMask::all(scene_->state.num_vertices());
    pacer_.reset();
    frame_ = 0;
    failed_ = false;
    last_step_ms_ = 0.0;
    mean_step_ms_ = 0.0;
}

// Timing is taken here, in the app; the engine itself never reads a clock.
void SimDriver::step() {
    if (failed_) return;
    const auto start = std::chrono::steady_clock::now();
    scene_->solver.step(scene_->state, mask_, kFrameDt);
    const auto stop = std::chrono::steady_clock::now();
    last_step_ms_ = std::chrono::duration<double, std::milli>(stop - start).count();
    mean_step_ms_ = frame_ == 0 ? last_step_ms_ : 0.9 * mean_step_ms_ + 0.1 * last_step_ms_;
    ++frame_;
    if (!all_finite(scene_->state)) failed_ = true;
}

int SimDriver::advance(double wall_dt) {
    const int steps = pacer_.steps_for(wall_dt);
    for (int i = 0; i < steps; ++i) step();
    return steps;
}

double SimDriver::sim_time() const {
    return static_cast<double>(frame_) * static_cast<double>(kFrameDt);
}

FrameMetrics SimDriver::metrics() const {
    const XpbdSolver& solver = scene_->solver;
    return {energy(scene_->state, solver.constraints(), solver.settings().gravity),
            penetration(scene_->state, solver.colliders()),
            constraint_error(scene_->state, solver.constraints())};
}

std::uint64_t SimDriver::state_hash() const {
    std::uint64_t h = 0xcbf29ce484222325ULL;
    fnv1a(h, scene_->state.positions);
    fnv1a(h, scene_->state.velocities);
    return h;
}

}  // namespace hp::viewer
