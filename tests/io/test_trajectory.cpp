#include <doctest/doctest.h>

#include <cstring>
#include <filesystem>
#include <initializer_list>
#include <fstream>
#include <iterator>
#include <stdexcept>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

#include "hp/io/trajectory.hpp"
#include "hp/metrics/frame_metrics.hpp"
#include "hp/scene/scenario.hpp"
#include "hp/scene/scene.hpp"

namespace {

namespace fs = std::filesystem;

const std::string kScenarioPath = HP_SCENARIO_DIR "/cube_drop.json";

std::string temp_path(const char* name) {
    return (fs::temp_directory_path() / name).string();
}

std::string read_text(const std::string& path) {
    std::ifstream in(path, std::ios::binary);
    return {std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>()};
}

hp::FrameMetrics measure(const hp::Scene& scene) {
    const hp::XpbdSolver& s = scene.solver;
    return hp::measure_frame(scene.state, s.constraints(), s.colliders(), s.settings().gravity);
}

// Records `frames` steps of cube_drop to `path`. Keeps every state so tests can compare them
// with what is read back. `finish` false leaves the file without end(), like a crash.
struct Recorded {
    std::vector<hp::SimState> states;
    std::vector<hp::FrameMetrics> metrics;
    std::uint64_t frame_size = 0;
};

Recorded record(const std::string& path, int frames, bool finish = true,
                std::uint32_t status = hp::kRecordingOk) {
    hp::RunInfo info;
    info.scenario_path = kScenarioPath;
    info.scenario_text = read_text(kScenarioPath);
    info.scenario = hp::load_scenario(kScenarioPath);
    info.machine = {{"label", "test"}, {"arch", "arm64"}};
    info.created_utc = "2026-01-01T00:00:00Z";
    hp::Scene scene = hp::build_scene(info.scenario);
    const hp::RegionMask mask = hp::RegionMask::all(scene.state.num_vertices());

    Recorded out;
    hp::TrajectoryWriter w(path);
    w.begin(info, scene);
    for (int f = 0; f <= frames; ++f) {
        if (f > 0) scene.solver.step(scene.state, mask, hp::kFrameDt);
        out.states.push_back(scene.state);
        out.metrics.push_back(measure(scene));
        // A fixed step time keeps the file deterministic for the byte comparison test.
        w.write_frame(f, scene.state, out.metrics.back(), f == 0 ? 0.0 : 1.5);
    }
    out.frame_size = w.frame_size();
    if (finish) w.end(status);
    return out;
}

template <typename T>
bool same_bytes(const std::vector<T>& a, const std::vector<T>& b) {
    return a.size() == b.size() && (a.empty() || std::memcmp(a.data(), b.data(),
                                                             a.size() * sizeof(T)) == 0);
}

}  // namespace

TEST_CASE("trajectory round trip is bit-exact") {
    const std::string path = temp_path("hp_test_roundtrip.hptraj");
    const Recorded rec = record(path, 12);
    const hp::Scene scene = hp::build_scene(hp::load_scenario(kScenarioPath));

    hp::TrajectoryReader r(path);
    CHECK(r.closed_cleanly());
    CHECK(r.status() == hp::kRecordingOk);
    CHECK(r.num_frames() == 13);
    CHECK(r.num_vertices() == scene.state.num_vertices());
    CHECK(r.frame_size() == rec.frame_size);
    // 112 bytes of scalars, 25 bytes per vertex, padded to 8.
    const auto n = static_cast<std::uint64_t>(scene.state.num_vertices());
    CHECK(r.frame_size() == (112 + 25 * n + 7) / 8 * 8);

    const hp::TrajectoryStatic& st = r.static_data();
    const hp::XpbdConstraints& c = scene.solver.constraints();
    CHECK(same_bytes(st.rest_positions, scene.state.rest_positions));
    CHECK(same_bytes(st.inv_mass, scene.state.inv_mass));
    CHECK(same_bytes(st.region_id, scene.state.region_id));
    CHECK(same_bytes(st.body_id, scene.state.body_id));
    CHECK(same_bytes(st.edges, c.edges));
    CHECK(same_bytes(st.edge_rest_length, c.edge_rest_length));
    CHECK(same_bytes(st.edge_compliance, c.edge_compliance));
    CHECK(same_bytes(st.tets, c.tets));
    CHECK(same_bytes(st.tet_rest_volume, c.tet_rest_volume));
    CHECK(same_bytes(st.tet_compliance, c.tet_compliance));
    CHECK(same_bytes(st.surface_triangles, scene.topology.surface_triangles));

    for (std::int64_t i = 0; i < r.num_frames(); ++i) {
        const hp::TrajectoryFrame f = r.read_frame(i);
        const auto k = static_cast<size_t>(i);
        CHECK(f.frame == i);
        CHECK(f.time == static_cast<double>(i) * static_cast<double>(hp::kFrameDt));
        CHECK(same_bytes(f.positions, rec.states[k].positions));
        CHECK(same_bytes(f.velocities, rec.states[k].velocities));
        CHECK(same_bytes(f.solver_owner, rec.states[k].solver_owner));
        CHECK(f.metrics.energy.kinetic == rec.metrics[k].energy.kinetic);
        CHECK(f.metrics.energy.gravitational == rec.metrics[k].energy.gravitational);
        CHECK(f.metrics.energy.elastic == rec.metrics[k].energy.elastic);
        CHECK(f.metrics.error.edge_max == rec.metrics[k].error.edge_max);
        CHECK(f.metrics.error.volume_rms == rec.metrics[k].error.volume_rms);
        CHECK(f.metrics.penetration.rms_depth == rec.metrics[k].penetration.rms_depth);
        CHECK(f.metrics.penetration.num_in_contact == rec.metrics[k].penetration.num_in_contact);
        CHECK(f.step_ms == (i == 0 ? 0.0 : 1.5));
    }
    CHECK_THROWS_AS(r.read_frame(13), std::out_of_range);

    // The header carries the run context.
    const std::string& h = r.header_json();
    CHECK(h.find("\"git_commit\"") != std::string::npos);
    CHECK(h.find("\"seed\"") != std::string::npos);
    CHECK(h.find("\"frame_layout\"") != std::string::npos);
    CHECK(h.find("\"label\": \"test\"") != std::string::npos);
    CHECK(h.find("only physical energy near convergence") != std::string::npos);
    fs::remove(path);
}

TEST_CASE("recording the same run twice gives identical files") {
    const std::string a = temp_path("hp_test_det_a.hptraj");
    const std::string b = temp_path("hp_test_det_b.hptraj");
    record(a, 20);
    record(b, 20);
    // Step time and timestamp are fixed by the test, so nothing else may differ.
    CHECK(read_text(a) == read_text(b));
    fs::remove(a);
    fs::remove(b);
}

TEST_CASE("unfinished and truncated trajectories") {
    const std::string path = temp_path("hp_test_partial.hptraj");

    SUBCASE("a run that never called end() is read up to its last whole frame") {
        const Recorded rec = record(path, 5, /*finish=*/false);
        {
            std::ofstream out(path, std::ios::binary | std::ios::app);
            const std::string half(rec.frame_size / 2, '\x7f');  // a frame cut off mid-write
            out << half;
        }
        hp::TrajectoryReader r(path);
        CHECK_FALSE(r.closed_cleanly());
        CHECK(r.status() == hp::kRecordingUnfinished);
        CHECK(r.num_frames() == 6);
        CHECK(same_bytes(r.read_frame(5).positions, rec.states[5].positions));
    }

    SUBCASE("a closed file with frames missing is rejected") {
        const Recorded rec = record(path, 5);
        fs::resize_file(path, fs::file_size(path) - rec.frame_size / 2);
        CHECK_THROWS_AS(hp::TrajectoryReader{path}, std::runtime_error);
    }

    SUBCASE("the producer's status code is kept") {
        record(path, 3, true, 3u);
        hp::TrajectoryReader r(path);
        CHECK(r.closed_cleanly());
        CHECK(r.status() == 3u);
        CHECK(r.num_frames() == 4);
    }
    fs::remove(path);
}

TEST_CASE("trajectory reader rejects files that are not trajectories") {
    const std::string path = temp_path("hp_test_bad.hptraj");
    record(path, 1);
    std::string bytes = read_text(path);

    SUBCASE("bad magic") { bytes[0] = 'X'; }
    SUBCASE("unknown version") { bytes[8] = 99; }
    SUBCASE("too short") { bytes.resize(20); }
    {
        std::ofstream out(path, std::ios::binary | std::ios::trunc);
        out << bytes;
    }
    CHECK_THROWS_AS(hp::TrajectoryReader{path}, std::runtime_error);
    CHECK_THROWS_AS(hp::TrajectoryReader{temp_path("hp_test_missing.hptraj")},
                    std::runtime_error);
    fs::remove(path);
}

namespace {

// Rewrites the JSON header of a recorded file with `edit` applied, keeping its length (compact
// JSON padded with spaces), so the preamble and every block offset stay valid.
template <typename Edit>
void edit_header(const std::string& path, Edit edit) {
    std::string bytes = read_text(path);
    std::uint32_t header_len = 0;
    std::memcpy(&header_len, bytes.data() + 12, sizeof(header_len));
    nlohmann::json h = nlohmann::json::parse(bytes.substr(64, header_len));
    edit(h);
    std::string text = h.dump();
    REQUIRE(text.size() <= header_len);
    text.resize(header_len, ' ');
    bytes.replace(64, header_len, text);
    std::ofstream out(path, std::ios::binary | std::ios::trunc);
    out << bytes;
}

// Sets `count` in the header and the first dimension of every static array sized by it.
void set_count(nlohmann::json& h, const char* count, std::int64_t value,
               std::initializer_list<const char*> arrays) {
    h["counts"][count] = value;
    for (auto& field : h["static_layout"]) {
        for (const char* name : arrays) {
            if (field["name"] == name) field["shape"][0] = value;
        }
    }
    if (std::string(count) == "vertices") {
        for (auto& field : h["frame_layout"]) {
            if (!field["shape"].empty()) field["shape"][0] = value;
        }
    }
}

void set_offset(nlohmann::json& h, const char* table, const char* name, nlohmann::json value) {
    for (auto& field : h[table]) {
        if (field["name"] == name) field["offset"] = value;
    }
}

}  // namespace

TEST_CASE("trajectory reader rejects corrupt header layouts with runtime_error") {
    const std::string path = temp_path("hp_test_corrupt_layout.hptraj");
    record(path, 1);
    // The unedited file must open, so the failures below come from the edits alone.
    edit_header(path, [](nlohmann::json&) {});
    CHECK_NOTHROW(hp::TrajectoryReader{path});

    SUBCASE("static offset that would wrap around 2^64") {
        edit_header(path, [](nlohmann::json& h) {
            set_offset(h, "static_layout", "rest_positions", 18446744073709551600ull);
        });
    }
    SUBCASE("frame offset that would wrap around 2^64") {
        edit_header(path, [](nlohmann::json& h) {
            set_offset(h, "frame_layout", "positions", 18446744073709551600ull);
        });
    }
    SUBCASE("negative offset") {
        edit_header(path, [](nlohmann::json& h) {
            set_offset(h, "static_layout", "edges", -64);
        });
    }
    SUBCASE("negative edge count and shapes") {
        edit_header(path, [](nlohmann::json& h) {
            set_count(h, "edges", -1, {"edges", "edge_rest_length", "edge_compliance"});
        });
    }
    SUBCASE("negative vertex count and shapes") {
        edit_header(path, [](nlohmann::json& h) {
            set_count(h, "vertices", -1,
                      {"rest_positions", "inv_mass", "region_id", "body_id"});
        });
    }
    SUBCASE("count beyond the int32 index range") {
        edit_header(path, [](nlohmann::json& h) {
            set_count(h, "tets", std::int64_t{1} << 40,
                      {"tets", "tet_rest_volume", "tet_compliance"});
        });
    }
    CHECK_THROWS_AS(hp::TrajectoryReader{path}, std::runtime_error);
    fs::remove(path);
}

TEST_CASE("trajectory writer enforces frame order and call order") {
    const std::string path = temp_path("hp_test_order.hptraj");
    hp::RunInfo info;
    info.scenario = hp::load_scenario(kScenarioPath);
    const hp::Scene scene = hp::build_scene(info.scenario);
    const hp::FrameMetrics m = measure(scene);

    hp::TrajectoryWriter w(path);
    CHECK_THROWS_AS(w.write_frame(0, scene.state, m, 0.0), std::logic_error);
    w.begin(info, scene);
    CHECK_THROWS_AS(w.begin(info, scene), std::logic_error);
    CHECK_THROWS_AS(w.write_frame(1, scene.state, m, 0.0), std::invalid_argument);
    w.write_frame(0, scene.state, m, 0.0);
    hp::SimState smaller = scene.state;
    smaller.positions.pop_back();
    CHECK_THROWS_AS(w.write_frame(1, smaller, m, 0.0), std::invalid_argument);
    w.end(hp::kRecordingOk);
    CHECK_THROWS_AS(w.write_frame(1, scene.state, m, 0.0), std::logic_error);
    CHECK(w.frames_written() == 1);

    CHECK_THROWS_AS(hp::TrajectoryWriter{"/nonexistent-dir/x.hptraj"}, std::runtime_error);
    fs::remove(path);
}
