#pragma once

#include <cstdint>
#include <limits>
#include <optional>

#include "hp/metrics/constraint_error.hpp"
#include "hp/metrics/energy.hpp"
#include "hp/metrics/penetration.hpp"
#include "hp/scene/scenario.hpp"
#include "hp/scene/scene.hpp"
#include "hp/solver/solver.hpp"

namespace hp::viewer {

// Turns elapsed wall time into a whole number of fixed simulation steps. Leftover time is
// carried to the next call. At most kMaxSteps are taken per call; time beyond that is dropped
// and behind() reports it, so a slow solver makes the simulation run slower than real time
// instead of falling further and further behind.
class FixedStepPacer {
public:
    static constexpr int kMaxSteps = 4;
    static constexpr double kMaxWallDt = 0.25;  // longer pauses (window drag, breakpoints) are cut

    explicit FixedStepPacer(double dt) : dt_(dt) {}

    // Number of steps to take for `wall_dt` seconds of wall time (negative counts as zero).
    int steps_for(double wall_dt);
    bool behind() const { return behind_; }
    void reset();

private:
    double dt_;
    double accumulator_ = 0.0;
    bool behind_ = false;
};

// Metrics of the current state, the same ones hp_run prints.
struct FrameMetrics {
    Energy energy;
    Penetration penetration;
    ConstraintError error;
};

// Owns a scene built from a scenario and steps it with the fixed frame dt, exactly as
// hp_run does, so frame N here is bit-identical to frame N there. Wall time only decides how
// many steps run per call; it never enters the physics.
class SimDriver {
public:
    // Builds the scene; throws as build_scene does.
    explicit SimDriver(Scenario scenario);

    // Advances one frame and times it. Does nothing once the state has become non-finite.
    void step();
    // Advances as many frames as `wall_dt` seconds of real time call for (see FixedStepPacer),
    // never going past frame `frame_limit`, and returns how many were taken.
    int advance(double wall_dt, Index frame_limit = std::numeric_limits<Index>::max());
    // Rebuilds the scene from the scenario, back to frame 0.
    void reset();

    const Scenario& scenario() const { return scenario_; }
    const Scene& scene() const { return *scene_; }
    Index frame() const { return frame_; }
    double sim_time() const;
    bool behind() const { return pacer_.behind(); }
    bool failed() const { return failed_; }  // the state became non-finite and stepping stopped
    double last_step_ms() const { return last_step_ms_; }
    double mean_step_ms() const { return mean_step_ms_; }  // exponential moving average

    FrameMetrics metrics() const;
    // FNV-1a over positions then velocities, the same hash hp_run prints.
    std::uint64_t state_hash() const;

private:
    Scenario scenario_;
    std::optional<Scene> scene_;  // Scene has no default constructor
    RegionMask mask_;
    FixedStepPacer pacer_;
    Index frame_ = 0;
    bool failed_ = false;
    double last_step_ms_ = 0.0;
    double mean_step_ms_ = 0.0;
};

}  // namespace hp::viewer
