#include <cstring>
#include <stdexcept>

#include <nlohmann/json.hpp>

#include "hp/io/trajectory.hpp"
#include "trajectory_layout.hpp"

namespace hp {

namespace {

using json = nlohmann::json;
using detail::FieldSpec;

[[noreturn]] void fail(const std::string& path, const std::string& why) {
    throw std::runtime_error(path + ": " + why);
}

// Looks up each expected field in the header's layout table by name, checks its dtype and
// shape, and takes its offset from the file. Returns the offsets in the expected order.
std::vector<std::uint64_t> resolve(const json& table, const std::vector<FieldSpec>& expected,
                                   std::uint64_t limit, const std::string& path,
                                   const char* what) {
    std::vector<std::uint64_t> offsets;
    for (const FieldSpec& e : expected) {
        const json* found = nullptr;
        for (const json& f : table) {
            if (f.at("name").get<std::string>() == e.name) found = &f;
        }
        if (found == nullptr) fail(path, std::string(what) + " has no field '" + e.name + "'");
        if (found->at("dtype").get<std::string>() != e.dtype ||
            found->at("shape").get<std::vector<std::int64_t>>() != e.shape) {
            fail(path, std::string(what) + " field '" + e.name + "' has an unexpected type");
        }
        const auto off = found->at("offset").get<std::uint64_t>();
        if (off + e.bytes() > limit) {
            fail(path, std::string(what) + " field '" + e.name + "' runs past its block");
        }
        offsets.push_back(off);
    }
    return offsets;
}

// Vec3 is three packed floats (checked in types.hpp), so raw copies are safe; the void*
// casts say so to the compiler.
template <typename T>
void read_array(const std::vector<unsigned char>& block, std::uint64_t off, size_t count,
                std::vector<T>& out) {
    out.resize(count);
    if (count > 0) std::memcpy(static_cast<void*>(out.data()), block.data() + off, count * sizeof(T));
}

}  // namespace

TrajectoryReader::TrajectoryReader(const std::string& path) : in_(path, std::ios::binary) {
    if (!in_) fail(path, "cannot open");
    in_.seekg(0, std::ios::end);
    const auto file_size = static_cast<std::uint64_t>(in_.tellg());
    in_.seekg(0);

    unsigned char pre[detail::kPreambleSize];
    if (file_size < detail::kPreambleSize ||
        !in_.read(reinterpret_cast<char*>(pre), sizeof(pre))) {
        fail(path, "too short to be a trajectory");
    }
    if (std::memcmp(pre, detail::kTrajMagic, sizeof(detail::kTrajMagic)) != 0) {
        fail(path, "not a trajectory file (bad magic)");
    }
    const auto version = detail::get<std::uint32_t>(pre, detail::kOffVersion);
    if (version != kTrajectoryFormatVersion) {
        fail(path, "unsupported format version " + std::to_string(version));
    }
    const auto header_len = detail::get<std::uint32_t>(pre, detail::kOffHeaderLen);
    const auto static_offset = detail::get<std::uint64_t>(pre, detail::kOffStatic);
    data_offset_ = detail::get<std::uint64_t>(pre, detail::kOffData);
    frame_size_ = detail::get<std::uint64_t>(pre, detail::kOffFrameSize);
    const auto frames_written = detail::get<std::uint64_t>(pre, detail::kOffFramesWritten);
    status_ = detail::get<std::uint32_t>(pre, detail::kOffStatus);
    if (static_offset < detail::kPreambleSize + header_len || data_offset_ < static_offset ||
        frame_size_ == 0 || data_offset_ > file_size) {
        fail(path, "inconsistent preamble or truncated header");
    }

    header_json_.resize(header_len);
    if (!in_.read(header_json_.data(), header_len)) fail(path, "truncated header");
    json h;
    try {
        h = json::parse(header_json_);
        const json& counts = h.at("counts");
        num_vertices_ = counts.at("vertices").get<Index>();
        const auto edges = counts.at("edges").get<std::int64_t>();
        const auto tets = counts.at("tets").get<std::int64_t>();
        const auto tris = counts.at("surface_triangles").get<std::int64_t>();

        std::uint64_t record_size = 0;
        offsets_ = resolve(h.at("frame_layout"), detail::frame_fields(num_vertices_, record_size),
                           frame_size_, path, "frame_layout");

        std::uint64_t static_size = 0;
        const std::vector<FieldSpec> sf =
            detail::static_fields(num_vertices_, edges, tets, tris, static_size);
        const std::vector<std::uint64_t> so =
            resolve(h.at("static_layout"), sf, data_offset_ - static_offset, path, "static_layout");

        std::vector<unsigned char> block(data_offset_ - static_offset);
        in_.seekg(static_cast<std::streamoff>(static_offset));
        if (!in_.read(reinterpret_cast<char*>(block.data()),
                      static_cast<std::streamsize>(block.size()))) {
            fail(path, "truncated static block");
        }
        const auto nv = static_cast<size_t>(num_vertices_);
        const auto ne = static_cast<size_t>(edges);
        const auto nt = static_cast<size_t>(tets);
        using namespace detail;
        read_array(block, so[kSRestPositions], nv, static_.rest_positions);
        read_array(block, so[kSInvMass], nv, static_.inv_mass);
        read_array(block, so[kSRegionId], nv, static_.region_id);
        read_array(block, so[kSBodyId], nv, static_.body_id);
        read_array(block, so[kSEdges], ne, static_.edges);
        read_array(block, so[kSEdgeRestLength], ne, static_.edge_rest_length);
        read_array(block, so[kSEdgeCompliance], ne, static_.edge_compliance);
        read_array(block, so[kSTets], nt, static_.tets);
        read_array(block, so[kSTetRestVolume], nt, static_.tet_rest_volume);
        read_array(block, so[kSTetCompliance], nt, static_.tet_compliance);
        read_array(block, so[kSSurfaceTriangles], static_cast<size_t>(tris),
                   static_.surface_triangles);
    } catch (const json::exception& e) {
        fail(path, std::string("bad header: ") + e.what());
    }

    // A cleanly closed file must hold every frame it claims; an unfinished one (crash or
    // missing end()) is read up to its last whole record.
    const std::uint64_t whole = (file_size - data_offset_) / frame_size_;
    if (closed_cleanly()) {
        if (frames_written > whole) fail(path, "truncated: fewer frames than the preamble says");
        num_frames_ = static_cast<std::int64_t>(frames_written);
    } else {
        num_frames_ = static_cast<std::int64_t>(whole);
    }
}

TrajectoryFrame TrajectoryReader::read_frame(std::int64_t index) {
    if (index < 0 || index >= num_frames_) throw std::out_of_range("trajectory frame index");
    std::vector<unsigned char> buf(frame_size_);
    in_.clear();
    in_.seekg(static_cast<std::streamoff>(data_offset_ +
                                          static_cast<std::uint64_t>(index) * frame_size_));
    if (!in_.read(reinterpret_cast<char*>(buf.data()), static_cast<std::streamsize>(buf.size()))) {
        throw std::runtime_error("cannot read trajectory frame");
    }

    using namespace detail;
    const std::vector<std::uint64_t>& o = offsets_;
    const unsigned char* r = buf.data();
    TrajectoryFrame f;
    f.frame = get<std::int64_t>(r, o[kFFrame]);
    f.time = get<double>(r, o[kFTime]);
    f.metrics.energy.kinetic = get<double>(r, o[kFKinetic]);
    f.metrics.energy.gravitational = get<double>(r, o[kFGravitational]);
    f.metrics.energy.elastic = get<double>(r, o[kFElastic]);
    f.metrics.error.edge_rms = get<double>(r, o[kFEdgeRms]);
    f.metrics.error.edge_max = get<double>(r, o[kFEdgeMax]);
    f.metrics.error.volume_rms = get<double>(r, o[kFVolumeRms]);
    f.metrics.error.volume_max = get<double>(r, o[kFVolumeMax]);
    f.metrics.penetration.max_depth = get<double>(r, o[kFMaxPenetration]);
    f.metrics.penetration.rms_depth = get<double>(r, o[kFRmsPenetration]);
    f.step_ms = get<double>(r, o[kFStepMs]);
    f.metrics.error.inverted_tets = get<std::int32_t>(r, o[kFInvertedTets]);
    f.metrics.penetration.num_penetrating = get<std::int32_t>(r, o[kFNumPenetrating]);
    f.metrics.penetration.num_in_contact = get<std::int32_t>(r, o[kFNumInContact]);
    const auto n = static_cast<size_t>(num_vertices_);
    f.positions.resize(n);
    f.velocities.resize(n);
    f.solver_owner.resize(n);
    if (n > 0) {
        std::memcpy(static_cast<void*>(f.positions.data()), r + o[kFPositions], n * sizeof(Vec3));
        std::memcpy(static_cast<void*>(f.velocities.data()), r + o[kFVelocities], n * sizeof(Vec3));
        std::memcpy(f.solver_owner.data(), r + o[kFSolverOwner], n);
    }
    return f;
}

}  // namespace hp
