#include <doctest/doctest.h>

#include <algorithm>
#include <cstring>
#include <filesystem>
#include <string>
#include <vector>

#include "hp/metrics/penetration.hpp"
#include "hp/scene/scene.hpp"
#include "hp/solver/xpbd.hpp"

// Every file in scenarios/ must load, build without warnings and run cleanly, so the
// examples cannot silently rot as the engine changes.

namespace {

// Same bound as the contact tests: 10 um at the end of any frame.
constexpr double kMaxPenetration = 1e-5;
constexpr int kFrames = 60;

std::vector<std::string> scenario_files() {
    std::vector<std::string> files;
    for (const auto& entry : std::filesystem::directory_iterator(HP_SCENARIO_DIR)) {
        if (entry.path().extension() == ".json") files.push_back(entry.path().string());
    }
    std::sort(files.begin(), files.end());
    return files;
}

bool finite(const hp::SimState& s) {
    for (size_t v = 0; v < s.positions.size(); ++v) {
        if (!s.positions[v].allFinite() || !s.velocities[v].allFinite()) return false;
    }
    return true;
}

}  // namespace

TEST_CASE("example scenarios load, run and stay out of colliders deterministically") {
    const std::vector<std::string> files = scenario_files();
    CHECK(files.size() >= 6);
    for (const std::string& path : files) {
        CAPTURE(path);
        const hp::Scenario scenario = hp::load_scenario(path);
        hp::Scene a = hp::build_scene(scenario);
        hp::Scene b = hp::build_scene(scenario);
        CHECK(a.warnings.empty());

        const hp::RegionMask mask = hp::RegionMask::all(a.state.num_vertices());
        const int frames = std::min(kFrames, static_cast<int>(scenario.frames));
        double max_depth = 0.0;
        for (int f = 0; f < frames; ++f) {
            a.solver.step(a.state, mask, hp::kFrameDt);
            b.solver.step(b.state, mask, hp::kFrameDt);
            REQUIRE(finite(a.state));
            max_depth =
                std::max(max_depth, hp::penetration(a.state, a.solver.colliders()).max_depth);
        }
        CHECK(max_depth <= kMaxPenetration);
        const size_t bytes = a.state.positions.size() * sizeof(hp::Vec3);
        CHECK(std::memcmp(a.state.positions.data(), b.state.positions.data(), bytes) == 0);
        CHECK(std::memcmp(a.state.velocities.data(), b.state.velocities.data(), bytes) == 0);
    }
}
