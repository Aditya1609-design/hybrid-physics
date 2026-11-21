#include <cstring>
#include <stdexcept>

#include <nlohmann/json.hpp>

#include "hp/io/trajectory.hpp"
#include "hp/solver/xpbd.hpp"
#include "hp/version.hpp"
#include "trajectory_layout.hpp"

namespace hp {

namespace {

using json = nlohmann::ordered_json;
using detail::FieldSpec;

json vec_json(const Vec3& v) {
    return json::array({static_cast<double>(v.x()), static_cast<double>(v.y()),
                        static_cast<double>(v.z())});
}

// Only the parameters that matter for each collider kind are written.
json collider_json(const Collider& c) {
    json j;
    j["friction"] = static_cast<double>(c.friction);
    switch (c.kind) {
        case ColliderKind::Plane:
            j["type"] = "plane";
            j["point"] = vec_json(c.center);
            j["normal"] = vec_json(c.normal);
            break;
        case ColliderKind::Sphere:
            j["type"] = "sphere";
            j["center"] = vec_json(c.center);
            j["radius"] = static_cast<double>(c.radius);
            break;
        case ColliderKind::Box: {
            j["type"] = "box";
            j["center"] = vec_json(c.center);
            j["half_extents"] = vec_json(c.half_extents);
            json rows = json::array();  // row-major; columns are the box axes
            for (int r = 0; r < 3; ++r) {
                rows.push_back(json::array({static_cast<double>(c.rotation(r, 0)),
                                            static_cast<double>(c.rotation(r, 1)),
                                            static_cast<double>(c.rotation(r, 2))}));
            }
            j["rotation"] = rows;
            break;
        }
    }
    return j;
}

json layout_json(const std::vector<FieldSpec>& fields) {
    json arr = json::array();
    for (const FieldSpec& f : fields) {
        arr.push_back({{"name", f.name}, {"dtype", f.dtype}, {"shape", f.shape},
                       {"offset", f.offset}});
    }
    return arr;
}

void write_all(std::FILE* f, const void* data, size_t bytes, const std::string& path) {
    if (bytes > 0 && std::fwrite(data, 1, bytes, f) != bytes) {
        throw std::runtime_error("cannot write " + path);
    }
}

// Copies a vector's raw bytes into the block at the field's offset.
template <typename T>
void copy_array(std::vector<unsigned char>& block, const FieldSpec& f, const std::vector<T>& v) {
    if (v.size() * sizeof(T) != f.bytes()) throw std::logic_error("static array size mismatch");
    if (!v.empty()) std::memcpy(block.data() + f.offset, v.data(), f.bytes());
}

}  // namespace

TrajectoryWriter::TrajectoryWriter(const std::string& path) : path_(path) {
    if (!detail::host_is_little_endian()) {
        throw std::runtime_error("trajectory files can only be written on little-endian hosts");
    }
    file_ = std::fopen(path.c_str(), "wb");
    if (file_ == nullptr) throw std::runtime_error("cannot write " + path);
}

TrajectoryWriter::~TrajectoryWriter() {
    if (file_ != nullptr) std::fclose(file_);
}

void TrajectoryWriter::begin(const RunInfo& info, const Scene& scene) {
    if (begun_) throw std::logic_error("TrajectoryWriter::begin called twice");
    if (file_ == nullptr) throw std::logic_error("TrajectoryWriter already closed");
    begun_ = true;

    const SimState& s = scene.state;
    const XpbdConstraints& c = scene.solver.constraints();
    num_vertices_ = s.num_vertices();
    const std::int64_t n = num_vertices_;
    const std::int64_t num_tris = static_cast<std::int64_t>(scene.topology.surface_triangles.size());

    std::uint64_t static_size = 0;
    const std::vector<FieldSpec> sf =
        detail::static_fields(n, c.num_edges(), c.num_tets(), num_tris, static_size);
    const std::vector<FieldSpec> ff = detail::frame_fields(n, frame_size_);

    const Scenario& sc = info.scenario;
    const XpbdSettings& set = scene.solver.settings();
    json h;
    h["format"] = "hptraj";
    h["format_version"] = kTrajectoryFormatVersion;
    h["endianness"] = "little";
    h["dt"] = static_cast<double>(kFrameDt);
    h["frames_planned"] = sc.frames;
    h["counts"] = {{"vertices", n},
                   {"edges", c.num_edges()},
                   {"tets", c.num_tets()},
                   {"surface_triangles", num_tris}};
    json colliders = json::array();
    for (const Collider& col : scene.solver.colliders()) colliders.push_back(collider_json(col));
    h["scenario"] = {{"path", info.scenario_path},
                     {"name", sc.name},
                     {"seed", sc.seed},
                     {"preset", sc.preset},
                     {"frames", sc.frames},
                     {"solver",
                      {{"substeps", set.substeps},
                       {"iterations", set.iterations},
                       {"gravity", vec_json(set.gravity)}}},
                     {"colliders", colliders},
                     {"text", info.scenario_text}};
    h["engine"] = {{"version", version()},
                   {"build_info", build_info()},
                   {"git_commit", git_commit()},
                   {"git_dirty", git_dirty()}};
    json machine = json::object();
    for (const auto& kv : info.machine) machine[kv.first] = kv.second;
    h["machine"] = machine;
    h["created_utc"] = info.created_utc;
    h["notes"] = {
        {"frame_zero", "frame 0 is the initial state, before any step"},
        {"elastic_energy",
         "elastic_j is sum C^2/(2 alpha) and is only physical energy near convergence "
         "(reference preset, or a body at rest); with the realtime preset it mostly measures "
         "unconverged constraint residual"},
        {"step_ms", "wall-clock time of the step; not deterministic, excluded from comparisons"},
        {"created_utc", "wall-clock metadata; not deterministic"}};
    h["static_layout"] = layout_json(sf);
    h["frame_layout"] = layout_json(ff);
    h["frame_size"] = frame_size_;

    const std::string header = h.dump(2);
    const std::uint64_t static_offset =
        detail::align_up(detail::kPreambleSize + header.size(), detail::kBlockAlign);
    const std::uint64_t data_offset = static_offset + static_size;

    unsigned char pre[detail::kPreambleSize] = {};
    std::memcpy(pre, detail::kTrajMagic, sizeof(detail::kTrajMagic));
    detail::put<std::uint32_t>(pre, detail::kOffVersion, kTrajectoryFormatVersion);
    detail::put<std::uint32_t>(pre, detail::kOffHeaderLen, static_cast<std::uint32_t>(header.size()));
    detail::put<std::uint64_t>(pre, detail::kOffStatic, static_offset);
    detail::put<std::uint64_t>(pre, detail::kOffData, data_offset);
    detail::put<std::uint64_t>(pre, detail::kOffFrameSize, frame_size_);
    detail::put<std::uint64_t>(pre, detail::kOffFramesWritten, 0);
    detail::put<std::uint32_t>(pre, detail::kOffStatus, kRecordingUnfinished);
    write_all(file_, pre, sizeof(pre), path_);
    write_all(file_, header.data(), header.size(), path_);
    const std::vector<unsigned char> pad(static_offset - detail::kPreambleSize - header.size(), 0);
    write_all(file_, pad.data(), pad.size(), path_);

    std::vector<unsigned char> block(static_size, 0);
    copy_array(block, sf[detail::kSRestPositions], s.rest_positions);
    copy_array(block, sf[detail::kSInvMass], s.inv_mass);
    copy_array(block, sf[detail::kSRegionId], s.region_id);
    copy_array(block, sf[detail::kSBodyId], s.body_id);
    copy_array(block, sf[detail::kSEdges], c.edges);
    copy_array(block, sf[detail::kSEdgeRestLength], c.edge_rest_length);
    copy_array(block, sf[detail::kSEdgeCompliance], c.edge_compliance);
    copy_array(block, sf[detail::kSTets], c.tets);
    copy_array(block, sf[detail::kSTetRestVolume], c.tet_rest_volume);
    copy_array(block, sf[detail::kSTetCompliance], c.tet_compliance);
    copy_array(block, sf[detail::kSSurfaceTriangles], scene.topology.surface_triangles);
    write_all(file_, block.data(), block.size(), path_);

    offsets_.clear();
    for (const FieldSpec& f : ff) offsets_.push_back(f.offset);
    record_.assign(frame_size_, 0);
}

void TrajectoryWriter::write_frame(Index frame, const SimState& state,
                                   const FrameMetrics& m, double step_ms) {
    if (!begun_ || file_ == nullptr) throw std::logic_error("TrajectoryWriter is not open");
    if (static_cast<std::uint64_t>(frame) != frames_written_) {
        throw std::invalid_argument("trajectory frames must be written in order from 0");
    }
    if (state.num_vertices() != num_vertices_ ||
        state.velocities.size() != state.positions.size() ||
        state.solver_owner.size() != state.positions.size()) {
        throw std::invalid_argument("state size differs from the recorded scene");
    }

    // Offsets come from the same table the header was written from.
    using namespace detail;
    const std::vector<std::uint64_t>& o = offsets_;
    const size_t n = static_cast<size_t>(num_vertices_);
    unsigned char* r = record_.data();
    put<std::int64_t>(r, o[kFFrame], frame);
    put<double>(r, o[kFTime], static_cast<double>(frame) * static_cast<double>(kFrameDt));
    put<double>(r, o[kFKinetic], m.energy.kinetic);
    put<double>(r, o[kFGravitational], m.energy.gravitational);
    put<double>(r, o[kFElastic], m.energy.elastic);
    put<double>(r, o[kFEdgeRms], m.error.edge_rms);
    put<double>(r, o[kFEdgeMax], m.error.edge_max);
    put<double>(r, o[kFVolumeRms], m.error.volume_rms);
    put<double>(r, o[kFVolumeMax], m.error.volume_max);
    put<double>(r, o[kFMaxPenetration], m.penetration.max_depth);
    put<double>(r, o[kFRmsPenetration], m.penetration.rms_depth);
    put<double>(r, o[kFStepMs], step_ms);
    put<std::int32_t>(r, o[kFInvertedTets], m.error.inverted_tets);
    put<std::int32_t>(r, o[kFNumPenetrating], m.penetration.num_penetrating);
    put<std::int32_t>(r, o[kFNumInContact], m.penetration.num_in_contact);
    if (n > 0) {
        std::memcpy(r + o[kFPositions], state.positions.data(), n * sizeof(Vec3));
        std::memcpy(r + o[kFVelocities], state.velocities.data(), n * sizeof(Vec3));
        static_assert(sizeof(SolverId) == 1, "solver_owner is stored as one byte per vertex");
        std::memcpy(r + o[kFSolverOwner], state.solver_owner.data(), n);
    }

    write_all(file_, r, record_.size(), path_);
    ++frames_written_;
}

void TrajectoryWriter::end(std::uint32_t status) {
    if (!begun_ || file_ == nullptr) throw std::logic_error("TrajectoryWriter is not open");
    unsigned char buf[8];
    detail::put<std::uint64_t>(buf, 0, frames_written_);
    bool ok = std::fseek(file_, static_cast<long>(detail::kOffFramesWritten), SEEK_SET) == 0 &&
              std::fwrite(buf, 1, 8, file_) == 8;
    detail::put<std::uint32_t>(buf, 0, status);
    ok = ok && std::fseek(file_, static_cast<long>(detail::kOffStatus), SEEK_SET) == 0 &&
         std::fwrite(buf, 1, 4, file_) == 4;
    ok = std::fclose(file_) == 0 && ok;
    file_ = nullptr;
    if (!ok) throw std::runtime_error("cannot finish " + path_);
}

}  // namespace hp
