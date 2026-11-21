#pragma once

#include <cstdint>
#include <cstdio>
#include <fstream>
#include <string>
#include <vector>

#include "hp/core/types.hpp"
#include "hp/io/recorder.hpp"
#include "hp/mesh/tet_mesh.hpp"
#include "hp/mesh/topology.hpp"

namespace hp {

// Version of the .hptraj format written by TrajectoryWriter (layout in docs/data_format.md).
constexpr std::uint32_t kTrajectoryFormatVersion = 1;

// Writes a run to a .hptraj file: fixed preamble, JSON header, static mesh block, then one
// fixed-size record per frame. Records are streamed, so memory use does not grow with the run.
class TrajectoryWriter final : public IRecorder {
public:
    // Opens (and truncates) the file. Throws std::runtime_error if it cannot be opened, or on
    // a big-endian machine, since the format is little-endian.
    explicit TrajectoryWriter(const std::string& path);
    // Closes the file. If end() was not called the file keeps the "unfinished" status, so a
    // reader can tell the run did not complete.
    ~TrajectoryWriter() override;

    TrajectoryWriter(const TrajectoryWriter&) = delete;
    TrajectoryWriter& operator=(const TrajectoryWriter&) = delete;

    // Throws std::logic_error if called twice, std::runtime_error on a write failure.
    void begin(const RunInfo& info, const Scene& scene) override;
    // Throws std::logic_error before begin() or after end(), std::invalid_argument if the
    // frame number is out of order or the state's size differs from the scene's.
    void write_frame(Index frame, const SimState& state, const FrameMetrics& metrics,
                     double step_ms) override;
    // Stores the frame count and status in the preamble and closes the file.
    void end(std::uint32_t status) override;

    std::uint64_t frame_size() const { return frame_size_; }
    std::uint64_t frames_written() const { return frames_written_; }

private:
    std::string path_;
    std::FILE* file_ = nullptr;
    bool begun_ = false;
    Index num_vertices_ = 0;
    std::uint64_t frame_size_ = 0;
    std::uint64_t frames_written_ = 0;
    std::vector<std::uint64_t> offsets_;  // byte offset of each frame record field
    std::vector<unsigned char> record_;   // reused buffer for one frame record
};

// Mesh and material data stored once per file.
struct TrajectoryStatic {
    std::vector<Vec3> rest_positions;
    std::vector<Real> inv_mass;
    std::vector<Index> region_id;
    std::vector<Index> body_id;
    std::vector<Edge> edges;
    std::vector<Real> edge_rest_length;
    std::vector<Real> edge_compliance;
    std::vector<Tet> tets;
    std::vector<Real> tet_rest_volume;
    std::vector<Real> tet_compliance;
    std::vector<Triangle> surface_triangles;
};

// One recorded frame.
struct TrajectoryFrame {
    std::int64_t frame = 0;
    double time = 0.0;
    FrameMetrics metrics;
    double step_ms = 0.0;
    std::vector<Vec3> positions;
    std::vector<Vec3> velocities;
    std::vector<SolverId> solver_owner;
};

// Reads a .hptraj file. Used by tests and C++ tools; Python has its own reader.
class TrajectoryReader {
public:
    // Reads the preamble, header and static block. Throws std::runtime_error if the file
    // cannot be read, is not a trajectory, has an unsupported version, is truncated, or its
    // header does not describe the expected fields.
    explicit TrajectoryReader(const std::string& path);

    const std::string& header_json() const { return header_json_; }
    std::uint32_t status() const { return status_; }
    // False if the writer never called end(); num_frames() then counts the whole records found.
    bool closed_cleanly() const { return status_ != kRecordingUnfinished; }
    std::int64_t num_frames() const { return num_frames_; }
    Index num_vertices() const { return num_vertices_; }
    std::uint64_t frame_size() const { return frame_size_; }
    const TrajectoryStatic& static_data() const { return static_; }

    // Throws std::out_of_range for a bad index, std::runtime_error on a read failure.
    TrajectoryFrame read_frame(std::int64_t index);

private:
    std::ifstream in_;
    std::string header_json_;
    std::uint32_t status_ = kRecordingUnfinished;
    std::int64_t num_frames_ = 0;
    Index num_vertices_ = 0;
    std::uint64_t data_offset_ = 0;
    std::uint64_t frame_size_ = 0;
    std::vector<std::uint64_t> offsets_;  // byte offset of each frame record field
    TrajectoryStatic static_;
};

}  // namespace hp
