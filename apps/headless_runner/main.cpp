// Runs a scenario file without graphics and prints metrics as it goes.
// Usage: hp_run <scenario.json> [--frames N] [--preset realtime|reference] [--every K]
//               [--csv metrics.csv] [--surface-obj final.obj] [--check]
//   --frames       run N frames instead of the scenario's count
//   --preset       replace the scenario's substep and iteration counts with a preset's
//   --every        print a metrics row every K frames (default: about ten rows per run)
//   --csv          write every frame's metrics and step time to a CSV file
//   --surface-obj  write the final surface as an OBJ file
//   --check        load and build the scene, print the summary and warnings, then stop
// Exit codes: 0 ok, 1 invalid scenario or unwritable output, 2 usage error, 3 the state
// became non-finite.
// Timing lives here, in the app; the engine itself never reads a clock.

#include <algorithm>
#include <chrono>
#include <cinttypes>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include "hp/core/state_hash.hpp"
#include "hp/io/obj.hpp"
#include "hp/metrics/frame_metrics.hpp"
#include "hp/scene/scenario.hpp"
#include "hp/scene/scene.hpp"
#include "hp/solver/xpbd.hpp"
#include "hp/version.hpp"

namespace {

struct Options {
    std::string scenario_path;
    long frames = -1;  // -1: use the scenario's count
    std::string preset;
    long every = 0;  // 0: choose from the frame count
    std::string csv_path;
    std::string obj_path;
    bool check_only = false;
};

int usage(const char* argv0) {
    std::fprintf(stderr,
                 "usage: %s <scenario.json> [--frames N] [--preset realtime|reference] "
                 "[--every K]\n"
                 "           [--csv metrics.csv] [--surface-obj final.obj] [--check]\n",
                 argv0);
    return 2;
}

// Positive integer argument, or -1 if the text is not one.
long parse_count(const char* text) {
    char* end = nullptr;
    const long n = std::strtol(text, &end, 10);
    return (end != text && *end == '\0' && n > 0) ? n : -1;
}

bool parse_args(int argc, char** argv, Options& o) {
    for (int i = 1; i < argc; ++i) {
        const char* a = argv[i];
        const bool has_value = i + 1 < argc;
        if (std::strcmp(a, "--frames") == 0 && has_value) {
            if ((o.frames = parse_count(argv[++i])) < 0) return false;
        } else if (std::strcmp(a, "--every") == 0 && has_value) {
            if ((o.every = parse_count(argv[++i])) < 0) return false;
        } else if (std::strcmp(a, "--preset") == 0 && has_value) {
            o.preset = argv[++i];
            if (o.preset != "realtime" && o.preset != "reference") return false;
        } else if (std::strcmp(a, "--csv") == 0 && has_value) {
            o.csv_path = argv[++i];
        } else if (std::strcmp(a, "--surface-obj") == 0 && has_value) {
            o.obj_path = argv[++i];
        } else if (std::strcmp(a, "--check") == 0) {
            o.check_only = true;
        } else if (a[0] != '-' && o.scenario_path.empty()) {
            o.scenario_path = a;
        } else {
            return false;
        }
    }
    return !o.scenario_path.empty();
}

bool all_finite(const hp::SimState& s) {
    for (size_t v = 0; v < s.positions.size(); ++v) {
        if (!s.positions[v].allFinite() || !s.velocities[v].allFinite()) return false;
    }
    return true;
}

// One frame's metrics, as printed and written to the CSV.
struct Row {
    long frame;
    hp::FrameMetrics m;
    double step_ms;
};

Row measure(const hp::Scene& scene, long frame, double step_ms) {
    const hp::XpbdSolver& solver = scene.solver;
    return {frame,
            hp::measure_frame(scene.state, solver.constraints(), solver.colliders(),
                              solver.settings().gravity),
            step_ms};
}

void print_header() {
    std::printf("%7s %8s %11s %11s %11s %10s %10s %10s %9s\n", "frame", "time_s", "kinetic_J",
                "gravity_J", "elastic_J", "edge_rms", "vol_rms", "max_pen_m", "step_ms");
}

void print_row(const Row& r) {
    std::printf("%7ld %8.3f %11.4g %11.4g %11.4g %10.3g %10.3g %10.3g %9.3f\n", r.frame,
                static_cast<double>(r.frame) * static_cast<double>(hp::kFrameDt),
                r.m.energy.kinetic, r.m.energy.gravitational, r.m.energy.elastic,
                r.m.error.edge_rms, r.m.error.volume_rms, r.m.penetration.max_depth, r.step_ms);
}

void write_csv_row(std::FILE* f, const Row& r) {
    std::fprintf(f, "%ld,%.6f,%.9g,%.9g,%.9g,%.6g,%.6g,%.6g,%.6g,%d,%.6g,%d,%d,%.4f\n", r.frame,
                 static_cast<double>(r.frame) * static_cast<double>(hp::kFrameDt),
                 r.m.energy.kinetic, r.m.energy.gravitational, r.m.energy.elastic,
                 r.m.error.edge_rms, r.m.error.edge_max, r.m.error.volume_rms,
                 r.m.error.volume_max, r.m.error.inverted_tets, r.m.penetration.max_depth,
                 r.m.penetration.num_penetrating, r.m.penetration.num_in_contact, r.step_ms);
}

}  // namespace

int main(int argc, char** argv) {
    Options opt;
    if (!parse_args(argc, argv, opt)) return usage(argv[0]);

    hp::Scenario scenario;
    try {
        scenario = hp::load_scenario(opt.scenario_path);
        if (opt.frames > 0) scenario.frames = static_cast<hp::Index>(opt.frames);
        if (!opt.preset.empty()) {
            // Only the counts change; gravity stays as the scenario set it.
            const hp::XpbdSettings p = opt.preset == "reference" ? hp::XpbdSettings::reference()
                                                                 : hp::XpbdSettings::realtime();
            scenario.solver.substeps = p.substeps;
            scenario.solver.iterations = p.iterations;
            scenario.preset = opt.preset;
        }
    } catch (const std::exception& e) {
        std::fprintf(stderr, "error: %s\n", e.what());
        return 1;
    }

    std::optional<hp::Scene> built;  // the scene has no default constructor
    try {
        built.emplace(hp::build_scene(scenario));
    } catch (const std::exception& e) {
        std::fprintf(stderr, "error: %s\n", e.what());
        return 1;
    }
    hp::Scene& scene = *built;

    hp::Index pinned = 0;
    for (const float w : scene.state.inv_mass) pinned += w == 0.0f ? 1 : 0;
    std::printf("build:     %s\n", hp::build_info());
    std::printf("scenario:  %s (%s), seed %" PRIu64 "\n", scenario.name.c_str(),
                opt.scenario_path.c_str(), scenario.seed);
    std::printf("mesh:      %d vertices, %d tets, %d edges, %d pinned\n",
                scene.state.num_vertices(), scene.mesh.num_tets(),
                scene.solver.constraints().num_edges(), pinned);
    std::printf("solver:    %s, %d substeps x %d iterations, dt %.6g s, %d frames\n",
                scenario.preset.c_str(), scenario.solver.substeps, scenario.solver.iterations,
                static_cast<double>(hp::kFrameDt), scenario.frames);
    std::printf("colliders: %zu\n", scene.solver.colliders().size());
    for (const std::string& w : scene.warnings) std::fprintf(stderr, "warning: %s\n", w.c_str());
    if (opt.check_only) {
        std::printf("check:     ok\n");
        return 0;
    }

    std::FILE* csv = nullptr;
    if (!opt.csv_path.empty()) {
        csv = std::fopen(opt.csv_path.c_str(), "w");
        if (csv == nullptr) {
            std::fprintf(stderr, "error: cannot write %s\n", opt.csv_path.c_str());
            return 1;
        }
        std::fprintf(csv, "# %s\n# scenario %s, seed %" PRIu64 ", preset %s\n",
                     hp::build_info(), scenario.name.c_str(), scenario.seed,
                     scenario.preset.c_str());
        std::fprintf(csv,
                     "frame,time_s,kinetic_j,gravitational_j,elastic_j,edge_rms,edge_max,"
                     "volume_rms,volume_max,inverted_tets,max_penetration_m,num_penetrating,"
                     "num_in_contact,step_ms\n");
    }

    const long frames = scenario.frames;
    const long every = opt.every > 0 ? opt.every : std::max(1L, frames / 10);
    std::printf("note:      elastic energy is only meaningful near convergence "
                "(reference preset, or at rest)\n\n");
    print_header();
    const Row start = measure(scene, 0, 0.0);
    print_row(start);
    if (csv != nullptr) write_csv_row(csv, start);

    const hp::RegionMask mask = hp::RegionMask::all(scene.state.num_vertices());
    std::vector<double> step_ms;
    step_ms.reserve(static_cast<size_t>(frames));
    double max_pen = start.m.penetration.max_depth;
    int status = 0;
    for (long f = 1; f <= frames; ++f) {
        const auto t0 = std::chrono::steady_clock::now();
        scene.solver.step(scene.state, mask, hp::kFrameDt);
        const auto t1 = std::chrono::steady_clock::now();
        step_ms.push_back(std::chrono::duration<double, std::milli>(t1 - t0).count());
        if (!all_finite(scene.state)) {
            std::fprintf(stderr, "error: state became non-finite at frame %ld\n", f);
            status = 3;
            break;
        }
        const Row row = measure(scene, f, step_ms.back());
        max_pen = std::max(max_pen, row.m.penetration.max_depth);
        if (f % every == 0 || f == frames) print_row(row);
        if (csv != nullptr) write_csv_row(csv, row);
    }
    if (csv != nullptr) std::fclose(csv);

    if (!opt.obj_path.empty()) {
        try {
            hp::write_surface_obj(opt.obj_path, scene.state.positions,
                                  scene.topology.surface_triangles);
        } catch (const std::exception& e) {
            std::fprintf(stderr, "error: %s\n", e.what());
            if (status == 0) status = 1;
        }
    }

    std::vector<double> sorted = step_ms;
    std::sort(sorted.begin(), sorted.end());
    const double median = sorted.empty() ? 0.0 : sorted[sorted.size() / 2];
    const double p95 =
        sorted.empty() ? 0.0 : sorted[std::min(sorted.size() - 1, (sorted.size() * 95) / 100)];
    std::printf("\nframes %zu, step median %.3f ms, p95 %.3f ms, max penetration %.3g m\n",
                step_ms.size(), median, p95, max_pen);
    std::printf("final state hash %016" PRIx64 "\n", hp::state_hash(scene.state));
    return status;
}
