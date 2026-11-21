#pragma once

// Fixed reference results for the example scenarios. Each record holds a fingerprint of the
// mesh and constraint data, state hashes at a few frames and the main metrics at the last
// frame. tests/regression/golden.json stores them; test_golden.cpp recomputes and compares
// them, and the hp_golden tool rewrites the file when a physics change has been approved.

#include <cstdint>
#include <string>
#include <utility>
#include <vector>

#include <nlohmann/json.hpp>

#include "hp/core/types.hpp"

namespace hp::golden {

// One scenario run: the scenario file name, the preset that replaces its substep and
// iteration counts (as hp_run --preset does), and the frames at which the state is hashed.
// The last checkpoint is the number of frames run.
struct Run {
    std::string scenario;  // file name inside the scenarios directory
    std::string preset;    // "realtime" or "reference"
    std::vector<Index> checkpoints;
};

struct Record {
    std::string scenario;
    std::string preset;
    std::uint64_t mesh_fingerprint = 0;
    std::vector<std::pair<Index, std::uint64_t>> hashes;  // (frame, state hash)
    std::vector<std::pair<std::string, double>> metrics;  // at the last checkpoint, fixed order
};

// Every run the golden file must contain: all scenarios on the real-time preset for 120
// frames, and on the reference preset for 10 frames.
std::vector<Run> runs(const std::string& scenario_dir);

// Builds the scenario, steps it to the last checkpoint and fills a record.
Record compute(const std::string& scenario_dir, const Run& run);

nlohmann::json to_json(const Record& record);
Record from_json(const nlohmann::json& j);

// "0123456789abcdef" form used by hp_run and the golden file.
std::string hex(std::uint64_t value);

}  // namespace hp::golden
