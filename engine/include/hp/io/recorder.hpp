#pragma once

#include <cstdint>
#include <string>
#include <utility>
#include <vector>

#include "hp/core/sim_state.hpp"
#include "hp/core/types.hpp"
#include "hp/metrics/frame_metrics.hpp"
#include "hp/scene/scenario.hpp"
#include "hp/scene/scene.hpp"

namespace hp {

// Status values stored when a recording is closed. Any other value is the producer's own
// error code (hp_run stores its exit code, e.g. 3 when the state became non-finite).
constexpr std::uint32_t kRecordingOk = 0;
constexpr std::uint32_t kRecordingUnfinished = 0xFFFFFFFFu;  // end() was never called

// Context for one run, supplied by the app. The engine never reads clocks or the machine, so
// the timestamp and machine description come from the caller and are stored as given.
struct RunInfo {
    std::string scenario_path;
    std::string scenario_text;  // the scenario file exactly as read
    Scenario scenario;          // after command-line overrides (frames, preset)
    std::vector<std::pair<std::string, std::string>> machine;  // key/value, in this order
    std::string created_utc;    // metadata only, never used by the physics
};

// Receives a run frame by frame. Frame 0 is the initial state, before any step; frames must
// arrive in order without gaps.
class IRecorder {
public:
    virtual ~IRecorder() = default;

    virtual void begin(const RunInfo& info, const Scene& scene) = 0;
    // `step_ms` is the wall time of the step that produced this frame (0 for frame 0).
    virtual void write_frame(Index frame, const SimState& state, const FrameMetrics& metrics,
                             double step_ms) = 0;
    virtual void end(std::uint32_t status) = 0;
};

}  // namespace hp
