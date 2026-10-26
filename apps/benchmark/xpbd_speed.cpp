// Times the XPBD presets per frame on box meshes of increasing size and prints a table.
// Usage: hp_xpbd_speed [--csv out.csv] [--quick] [--colliders]
//   --quick      only the three smallest meshes (for a fast sanity check)
//   --colliders  also time each preset with a ground plane, a sphere and a box, to show the
//                contact overhead (rows labelled "<preset>+colliders")
// Timing lives here, in the app; the engine itself never reads a clock.

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

#include "hp/mesh/generators.hpp"
#include "hp/scene/make_state.hpp"
#include "hp/scene/pin.hpp"
#include "hp/solver/xpbd.hpp"
#include "hp/version.hpp"

namespace {

struct Result {
    hp::Index vertices, tets, edges;
    const char* preset;
    hp::Index substeps;
    int frames;
    double median_ms, p95_ms;
};

// Median and 95th percentile of the frame times, in milliseconds.
void percentiles(std::vector<double> t, double& median, double& p95) {
    std::sort(t.begin(), t.end());
    median = t[t.size() / 2];
    p95 = t[std::min(t.size() - 1, (t.size() * 95) / 100)];
}

// Ground just below the sagging box, a sphere under its free end and a box beside it, so
// some vertices are in contact while timing.
std::vector<hp::Collider> benchmark_colliders() {
    return {hp::Collider::plane(hp::Vec3(0, -0.55f, 0), hp::Vec3::UnitY(), 0.5f),
            hp::Collider::sphere(hp::Vec3(0.5f, -0.9f, 0), 0.4f, 0.5f),
            hp::Collider::box(hp::Vec3(0, 0, 1.0f), hp::Vec3(0.5f, 0.5f, 0.3f),
                              hp::Mat3::Identity(), 0.5f)};
}

// Unit box pinned on its x = min face, so the constraints are under load while timing.
Result time_preset(hp::Index cells, const char* name, const hp::XpbdSettings& settings,
                   int frames, const std::vector<hp::Collider>& colliders) {
    hp::BoxParams params;
    params.cells_x = params.cells_y = params.cells_z = cells;
    const hp::TetMesh mesh = hp::make_box(params);
    const hp::Material material;
    hp::XpbdSolver solver(hp::build_constraints(mesh, hp::build_topology(mesh), material),
                          material, settings, colliders);
    hp::SimState state = hp::make_state(mesh, material.density);
    hp::pin_vertices(state, [](const hp::Vec3& p) { return p.x() < -0.5f + 1e-4f; });
    const hp::RegionMask mask = hp::RegionMask::all(state.num_vertices());

    const int warmup = std::max(2, frames / 10);
    for (int f = 0; f < warmup; ++f) solver.step(state, mask, hp::kFrameDt);

    std::vector<double> ms;
    ms.reserve(static_cast<size_t>(frames));
    for (int f = 0; f < frames; ++f) {
        const auto t0 = std::chrono::steady_clock::now();
        solver.step(state, mask, hp::kFrameDt);
        const auto t1 = std::chrono::steady_clock::now();
        ms.push_back(std::chrono::duration<double, std::milli>(t1 - t0).count());
    }
    Result r{mesh.num_vertices(), mesh.num_tets(), solver.constraints().num_edges(),
             name, settings.substeps, frames, 0.0, 0.0};
    percentiles(ms, r.median_ms, r.p95_ms);
    return r;
}

}  // namespace

int main(int argc, char** argv) {
    std::string csv_path;
    bool quick = false;
    bool with_colliders = false;
    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "--csv") == 0 && i + 1 < argc) {
            csv_path = argv[++i];
        } else if (std::strcmp(argv[i], "--quick") == 0) {
            quick = true;
        } else if (std::strcmp(argv[i], "--colliders") == 0) {
            with_colliders = true;
        } else {
            std::fprintf(stderr, "usage: %s [--csv out.csv] [--quick] [--colliders]\n",
                         argv[0]);
            return 2;
        }
    }

    // Box cells per side: 729 to 50653 vertices.
    std::vector<hp::Index> cells = {8, 10, 12, 16, 21, 27, 36};
    if (quick) cells.resize(3);

    std::printf("build: %s\n", hp::build_info());
    std::printf("%8s %8s %8s %20s %5s %7s %11s %11s %12s %13s\n", "verts", "tets", "edges",
                "preset", "sub", "frames", "median_ms", "p95_ms", "us/substep",
                "ns/constraint");

    std::vector<Result> results;
    for (const hp::Index c : cells) {
        const hp::Index verts = (c + 1) * (c + 1) * (c + 1);
        // Fewer frames for big meshes so the reference preset finishes in reasonable time.
        const int rt_frames = verts > 20000 ? 60 : 200;
        const int ref_frames = verts > 20000 ? 5 : (verts > 4000 ? 10 : 30);
        const size_t first = results.size();
        results.push_back(
            time_preset(c, "realtime", hp::XpbdSettings::realtime(), rt_frames, {}));
        results.push_back(
            time_preset(c, "reference", hp::XpbdSettings::reference(), ref_frames, {}));
        if (with_colliders) {
            const std::vector<hp::Collider> colliders = benchmark_colliders();
            results.push_back(time_preset(c, "realtime+colliders", hp::XpbdSettings::realtime(),
                                          rt_frames, colliders));
            results.push_back(time_preset(c, "reference+colliders",
                                          hp::XpbdSettings::reference(), ref_frames, colliders));
        }
        for (size_t k = first; k < results.size(); ++k) {
            const Result& r = results[k];
            const double per_sub_us = 1000.0 * r.median_ms / r.substeps;
            const double per_constraint_ns = 1000.0 * per_sub_us / (r.edges + r.tets);
            std::printf("%8d %8d %8d %20s %5d %7d %11.3f %11.3f %12.1f %13.2f\n", r.vertices,
                        r.tets, r.edges, r.preset, r.substeps, r.frames, r.median_ms, r.p95_ms,
                        per_sub_us, per_constraint_ns);
            std::fflush(stdout);
        }
    }

    if (!csv_path.empty()) {
        std::FILE* f = std::fopen(csv_path.c_str(), "w");
        if (!f) {
            std::fprintf(stderr, "cannot write %s\n", csv_path.c_str());
            return 1;
        }
        std::fprintf(f, "# %s\n", hp::build_info());
        std::fprintf(f, "vertices,tets,edges,preset,substeps,iterations,frames,median_ms,p95_ms\n");
        for (const Result& r : results) {
            std::fprintf(f, "%d,%d,%d,%s,%d,1,%d,%.4f,%.4f\n", r.vertices, r.tets, r.edges,
                         r.preset, r.substeps, r.frames, r.median_ms, r.p95_ms);
        }
        std::fclose(f);
    }
    return 0;
}
