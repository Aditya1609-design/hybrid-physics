// Computes the golden reference results for every example scenario.
// Usage: hp_golden [--write golden.json]
//   no option  print each run's final hash and compare with the stored file (exit 1 if any differ)
//   --write    replace the file with freshly computed results; do this only after a physics
//              change has been reviewed and approved, and commit the diff with that change
// The golden file and scenario directory are fixed at build time.

#include <cstdio>
#include <cstring>
#include <exception>
#include <fstream>
#include <map>
#include <string>

#include "golden.hpp"
#include "hp/version.hpp"

namespace {

using hp::golden::Record;

std::map<std::string, Record> read_file(const std::string& path) {
    std::ifstream in(path);
    std::map<std::string, Record> out;
    if (!in) return out;
    const nlohmann::json doc = nlohmann::json::parse(in);
    for (const auto& j : doc.at("runs")) {
        const Record r = hp::golden::from_json(j);
        out[r.scenario + "/" + r.preset] = r;
    }
    return out;
}

}  // namespace

int main(int argc, char** argv) {
    const std::string scenario_dir = HP_SCENARIO_DIR;
    std::string path = HP_GOLDEN_FILE;
    bool write = false;
    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "--write") == 0 && i + 1 < argc) {
            write = true;
            path = argv[++i];
        } else {
            std::fprintf(stderr, "usage: %s [--write golden.json]\n", argv[0]);
            return 2;
        }
    }

    try {
        const std::map<std::string, Record> stored = read_file(path);
        nlohmann::json runs = nlohmann::json::array();
        int differ = 0;
        for (const hp::golden::Run& run : hp::golden::runs(scenario_dir)) {
            const Record r = hp::golden::compute(scenario_dir, run);
            runs.push_back(hp::golden::to_json(r));
            const auto it = stored.find(r.scenario + "/" + r.preset);
            const bool same = it != stored.end() &&
                              it->second.mesh_fingerprint == r.mesh_fingerprint &&
                              it->second.hashes == r.hashes;
            if (!same) ++differ;
            std::printf("%-22s %-9s frame %3d  %s  %s\n", r.scenario.c_str(), r.preset.c_str(),
                        r.hashes.back().first, hp::golden::hex(r.hashes.back().second).c_str(),
                        same ? "same" : (it == stored.end() ? "NEW" : "DIFFERENT"));
        }
        if (write) {
            nlohmann::json doc = {
                {"description",
                 "Golden reference results for scenarios/*.json. Regenerate only with "
                 "hp_golden --write after an approved physics change."},
                {"engine_version", hp::version()},
                {"build_info", hp::build_info()},
                {"runs", runs}};
            std::ofstream out(path);
            out << doc.dump(2) << "\n";
            if (!out) throw std::runtime_error("cannot write " + path);
            std::printf("wrote %s (%d run(s) changed)\n", path.c_str(), differ);
            return 0;
        }
        if (differ == 0) {
            std::printf("all runs match %s\n", path.c_str());
            return 0;
        }
        std::printf("%d run(s) differ from %s\n", differ, path.c_str());
        return 1;
    } catch (const std::exception& e) {
        std::fprintf(stderr, "error: %s\n", e.what());
        return 1;
    }
}
