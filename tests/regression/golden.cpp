#include "golden.hpp"

#include <algorithm>
#include <cinttypes>
#include <cstdio>
#include <filesystem>
#include <stdexcept>

#include "hp/core/state_hash.hpp"
#include "hp/mesh/graph.hpp"
#include "hp/metrics/frame_metrics.hpp"
#include "hp/scene/scenario.hpp"
#include "hp/scene/scene.hpp"

namespace hp::golden {

namespace {

// FNV-1a over the raw bytes of an array, continuing from `h` (same scheme as state_hash).
template <typename T>
void fold(std::uint64_t& h, const std::vector<T>& data) {
    const auto* bytes = reinterpret_cast<const unsigned char*>(data.data());
    for (size_t i = 0; i < data.size() * sizeof(T); ++i) {
        h ^= bytes[i];
        h *= 0x100000001b3ULL;
    }
}

// Covers everything the scene builder derives from the scenario before the first step:
// mesh, topology, masses, constraint data and the GNN graph view.
std::uint64_t mesh_fingerprint(const Scene& scene) {
    std::uint64_t h = 0xcbf29ce484222325ULL;
    fold(h, scene.mesh.rest_positions);
    fold(h, scene.mesh.tets);
    fold(h, scene.topology.edges);
    fold(h, scene.topology.neighbor_offsets);
    fold(h, scene.topology.neighbors);
    fold(h, scene.topology.surface_triangles);
    fold(h, scene.state.inv_mass);
    const XpbdConstraints& c = scene.solver.constraints();
    fold(h, c.edge_rest_length);
    fold(h, c.edge_compliance);
    fold(h, c.tet_rest_volume);
    fold(h, c.tet_compliance);
    const MeshGraph g = build_graph(scene.mesh, scene.topology);
    fold(h, g.senders);
    fold(h, g.receivers);
    fold(h, g.rest_lengths);
    fold(h, g.rel_rest_pos);
    return h;
}

std::uint64_t parse_hex(const std::string& text) {
    std::size_t used = 0;
    const unsigned long long v = std::stoull(text, &used, 16);
    if (used != text.size() || text.size() != 16) {
        throw std::invalid_argument("golden: bad hash '" + text + "'");
    }
    return v;
}

}  // namespace

std::string hex(std::uint64_t value) {
    char buf[17];
    std::snprintf(buf, sizeof(buf), "%016" PRIx64, value);
    return buf;
}

std::vector<Run> runs(const std::string& scenario_dir) {
    std::vector<std::string> names;
    for (const auto& entry : std::filesystem::directory_iterator(scenario_dir)) {
        if (entry.path().extension() == ".json") names.push_back(entry.path().filename().string());
    }
    std::sort(names.begin(), names.end());
    std::vector<Run> out;
    for (const std::string& n : names) out.push_back({n, "realtime", {0, 1, 30, 60, 120}});
    for (const std::string& n : names) out.push_back({n, "reference", {0, 1, 5, 10}});
    return out;
}

Record compute(const std::string& scenario_dir, const Run& run) {
    Scenario scenario = load_scenario(scenario_dir + "/" + run.scenario);
    // Same override as hp_run --preset: only the counts change, gravity stays.
    const XpbdSettings p =
        run.preset == "reference" ? XpbdSettings::reference() : XpbdSettings::realtime();
    scenario.solver.substeps = p.substeps;
    scenario.solver.iterations = p.iterations;
    scenario.frames = run.checkpoints.back();
    scenario.preset = run.preset;

    Scene scene = build_scene(scenario);
    Record r;
    r.scenario = run.scenario;
    r.preset = run.preset;
    r.mesh_fingerprint = mesh_fingerprint(scene);

    const RegionMask mask = RegionMask::all(scene.state.num_vertices());
    Index frame = 0;
    for (const Index checkpoint : run.checkpoints) {
        for (; frame < checkpoint; ++frame) scene.solver.step(scene.state, mask, kFrameDt);
        r.hashes.emplace_back(frame, state_hash(scene.state));
    }

    const XpbdSolver& s = scene.solver;
    const FrameMetrics m =
        measure_frame(scene.state, s.constraints(), s.colliders(), s.settings().gravity);
    Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
    for (const Vec3& x : scene.state.positions) centroid += x.cast<double>();
    centroid /= static_cast<double>(scene.state.num_vertices());
    r.metrics = {
        {"kinetic_j", m.energy.kinetic},
        {"gravitational_j", m.energy.gravitational},
        {"elastic_j", m.energy.elastic},
        {"edge_rms", m.error.edge_rms},
        {"edge_max", m.error.edge_max},
        {"volume_rms", m.error.volume_rms},
        {"volume_max", m.error.volume_max},
        {"inverted_tets", static_cast<double>(m.error.inverted_tets)},
        {"max_penetration_m", m.penetration.max_depth},
        {"rms_penetration_m", m.penetration.rms_depth},
        {"num_penetrating", static_cast<double>(m.penetration.num_penetrating)},
        {"num_in_contact", static_cast<double>(m.penetration.num_in_contact)},
        {"centroid_x", centroid.x()},
        {"centroid_y", centroid.y()},
        {"centroid_z", centroid.z()},
    };
    return r;
}

nlohmann::json to_json(const Record& r) {
    nlohmann::json hashes = nlohmann::json::array();
    for (const auto& [frame, h] : r.hashes) hashes.push_back({{"frame", frame}, {"hash", hex(h)}});
    nlohmann::json metrics = nlohmann::json::object();
    for (const auto& [name, value] : r.metrics) metrics[name] = value;
    return {{"scenario", r.scenario},
            {"preset", r.preset},
            {"mesh_fingerprint", hex(r.mesh_fingerprint)},
            {"state_hashes", hashes},
            {"final_metrics", metrics}};
}

Record from_json(const nlohmann::json& j) {
    Record r;
    r.scenario = j.at("scenario").get<std::string>();
    r.preset = j.at("preset").get<std::string>();
    r.mesh_fingerprint = parse_hex(j.at("mesh_fingerprint").get<std::string>());
    for (const auto& h : j.at("state_hashes")) {
        r.hashes.emplace_back(h.at("frame").get<Index>(), parse_hex(h.at("hash").get<std::string>()));
    }
    for (const auto& [name, value] : j.at("final_metrics").items()) {
        r.metrics.emplace_back(name, value.get<double>());
    }
    return r;
}

}  // namespace hp::golden
