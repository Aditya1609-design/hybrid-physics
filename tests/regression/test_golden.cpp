#include <doctest/doctest.h>

#include <cmath>
#include <fstream>
#include <map>
#include <set>
#include <sstream>
#include <string>

#include "golden.hpp"

// Any change to the physics, the mesh generators, the scene builder or the constraint setup
// changes at least one state hash here. Hashes are compared exactly; the metric table printed
// on a mismatch only explains what moved. To accept an approved change, regenerate the file
// with hp_golden --write and commit the diff together with the change.

namespace {

std::map<std::string, hp::golden::Record> load_golden() {
    std::ifstream in(HP_GOLDEN_FILE);
    REQUIRE_MESSAGE(in.good(), "cannot read " HP_GOLDEN_FILE);
    const nlohmann::json doc = nlohmann::json::parse(in);
    std::map<std::string, hp::golden::Record> out;
    for (const auto& j : doc.at("runs")) {
        const hp::golden::Record r = hp::golden::from_json(j);
        out[r.scenario + "/" + r.preset] = r;
    }
    return out;
}

// Old value, new value and relative change of every final metric.
std::string metric_table(const hp::golden::Record& stored, const hp::golden::Record& now) {
    std::map<std::string, double> old_values(stored.metrics.begin(), stored.metrics.end());
    std::ostringstream out;
    out.precision(10);
    for (const auto& [name, value] : now.metrics) {
        const auto it = old_values.find(name);
        if (it == old_values.end()) {
            out << "  " << name << ": missing from golden file, now " << value << "\n";
            continue;
        }
        const double scale = std::max(std::abs(it->second), 1e-300);
        out << "  " << name << ": " << it->second << " -> " << value << " (rel change "
            << std::abs(value - it->second) / scale << ")\n";
    }
    return out.str();
}

void check_preset(const std::string& preset) {
    const std::map<std::string, hp::golden::Record> golden = load_golden();
    for (const hp::golden::Run& run : hp::golden::runs(HP_SCENARIO_DIR)) {
        if (run.preset != preset) continue;
        CAPTURE(run.scenario);
        const auto it = golden.find(run.scenario + "/" + run.preset);
        REQUIRE_MESSAGE(it != golden.end(), "no golden record; run hp_golden --write");
        const hp::golden::Record& stored = it->second;
        const hp::golden::Record now = hp::golden::compute(HP_SCENARIO_DIR, run);

        CHECK_MESSAGE(now.mesh_fingerprint == stored.mesh_fingerprint,
                      "mesh or constraint data changed: " << hp::golden::hex(stored.mesh_fingerprint)
                                                          << " -> "
                                                          << hp::golden::hex(now.mesh_fingerprint));
        bool hashes_match = now.hashes.size() == stored.hashes.size();
        for (size_t i = 0; hashes_match && i < now.hashes.size(); ++i) {
            hashes_match = now.hashes[i] == stored.hashes[i];
        }
        if (!hashes_match) {
            std::ostringstream detail;
            for (size_t i = 0; i < now.hashes.size(); ++i) {
                detail << "  frame " << now.hashes[i].first << ": "
                       << (i < stored.hashes.size() ? hp::golden::hex(stored.hashes[i].second)
                                                    : std::string("(none)"))
                       << " -> " << hp::golden::hex(now.hashes[i].second) << "\n";
            }
            FAIL_CHECK("state hashes changed\n" << detail.str() << "final metrics:\n"
                                                << metric_table(stored, now));
        }
    }
}

}  // namespace

TEST_CASE("golden file covers exactly the expected runs") {
    const std::map<std::string, hp::golden::Record> golden = load_golden();
    std::set<std::string> expected;
    for (const hp::golden::Run& run : hp::golden::runs(HP_SCENARIO_DIR)) {
        expected.insert(run.scenario + "/" + run.preset);
    }
    std::set<std::string> stored;
    for (const auto& entry : golden) stored.insert(entry.first);
    CHECK(stored == expected);
    CHECK(expected.size() == 12);  // six scenarios, two presets
}

TEST_CASE("golden results: real-time preset, 120 frames") { check_preset("realtime"); }

TEST_CASE("golden results: reference preset, 10 frames") { check_preset("reference"); }
