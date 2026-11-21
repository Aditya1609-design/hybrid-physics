#pragma once

// Byte layout of the .hptraj format, shared by the writer and the reader.
// docs/data_format.md is the full description; keep the two in sync.

#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

#include "hp/core/types.hpp"

namespace hp::detail {

constexpr char kTrajMagic[8] = {'H', 'P', 'T', 'R', 'A', 'J', '\0', '\0'};
constexpr std::uint64_t kPreambleSize = 64;
constexpr std::uint64_t kBlockAlign = 64;   // header end, static arrays and frame data start
constexpr std::uint64_t kRecordAlign = 8;   // frame record size is a multiple of this

// Preamble field offsets.
constexpr std::uint64_t kOffVersion = 8;         // u32
constexpr std::uint64_t kOffHeaderLen = 12;      // u32
constexpr std::uint64_t kOffStatic = 16;         // u64
constexpr std::uint64_t kOffData = 24;           // u64
constexpr std::uint64_t kOffFrameSize = 32;      // u64
constexpr std::uint64_t kOffFramesWritten = 40;  // u64, rewritten by end()
constexpr std::uint64_t kOffStatus = 48;         // u32, rewritten by end()

inline std::uint64_t align_up(std::uint64_t x, std::uint64_t a) { return (x + a - 1) / a * a; }

// One array in the static block or one field of a frame record. dtype uses numpy's notation
// so the Python reader can build its dtype straight from the header.
struct FieldSpec {
    std::string name;
    std::string dtype;
    std::uint64_t elem_size = 0;
    std::vector<std::int64_t> shape;  // empty for a scalar
    std::uint64_t offset = 0;         // from the static block start, or the record start

    std::uint64_t bytes() const {
        std::uint64_t n = elem_size;
        for (const std::int64_t d : shape) n *= static_cast<std::uint64_t>(d);
        return n;
    }
};

// Frame record fields in storage order; the enum indexes the vector from frame_fields().
enum FrameField : int {
    kFFrame, kFTime,
    kFKinetic, kFGravitational, kFElastic,
    kFEdgeRms, kFEdgeMax, kFVolumeRms, kFVolumeMax,
    kFMaxPenetration, kFRmsPenetration, kFStepMs,
    kFInvertedTets, kFNumPenetrating, kFNumInContact,
    kFPositions, kFVelocities, kFSolverOwner,
    kNumFrameFields
};

// Static block arrays in storage order.
enum StaticField : int {
    kSRestPositions, kSInvMass, kSRegionId, kSBodyId,
    kSEdges, kSEdgeRestLength, kSEdgeCompliance,
    kSTets, kSTetRestVolume, kSTetCompliance,
    kSSurfaceTriangles,
    kNumStaticFields
};

// Fields of one frame record for `n` vertices, packed in order (a 4-byte pad after the
// integer counters keeps the arrays 8-byte aligned). `record_size` gets the padded size.
inline std::vector<FieldSpec> frame_fields(std::int64_t n, std::uint64_t& record_size) {
    std::vector<FieldSpec> f = {
        {"frame", "<i8", 8, {}, 0},
        {"time", "<f8", 8, {}, 0},
        {"kinetic_j", "<f8", 8, {}, 0},
        {"gravitational_j", "<f8", 8, {}, 0},
        {"elastic_j", "<f8", 8, {}, 0},
        {"edge_rms", "<f8", 8, {}, 0},
        {"edge_max", "<f8", 8, {}, 0},
        {"volume_rms", "<f8", 8, {}, 0},
        {"volume_max", "<f8", 8, {}, 0},
        {"max_penetration_m", "<f8", 8, {}, 0},
        {"rms_penetration_m", "<f8", 8, {}, 0},
        {"step_ms", "<f8", 8, {}, 0},
        {"inverted_tets", "<i4", 4, {}, 0},
        {"num_penetrating", "<i4", 4, {}, 0},
        {"num_in_contact", "<i4", 4, {}, 0},
        {"positions", "<f4", 4, {n, 3}, 0},
        {"velocities", "<f4", 4, {n, 3}, 0},
        {"solver_owner", "<u1", 1, {n}, 0},
    };
    std::uint64_t off = 0;
    for (FieldSpec& s : f) {
        if (s.name == "positions") off = align_up(off, 8);  // skips the pad after the counters
        s.offset = off;
        off += s.bytes();
    }
    record_size = align_up(off, kRecordAlign);
    return f;
}

// Static block arrays, each starting on a 64-byte boundary. `block_size` gets the padded size.
inline std::vector<FieldSpec> static_fields(std::int64_t n, std::int64_t edges, std::int64_t tets,
                                            std::int64_t tris, std::uint64_t& block_size) {
    std::vector<FieldSpec> f = {
        {"rest_positions", "<f4", 4, {n, 3}, 0},
        {"inv_mass", "<f4", 4, {n}, 0},
        {"region_id", "<i4", 4, {n}, 0},
        {"body_id", "<i4", 4, {n}, 0},
        {"edges", "<i4", 4, {edges, 2}, 0},
        {"edge_rest_length", "<f4", 4, {edges}, 0},
        {"edge_compliance", "<f4", 4, {edges}, 0},
        {"tets", "<i4", 4, {tets, 4}, 0},
        {"tet_rest_volume", "<f4", 4, {tets}, 0},
        {"tet_compliance", "<f4", 4, {tets}, 0},
        {"surface_triangles", "<i4", 4, {tris, 3}, 0},
    };
    std::uint64_t off = 0;
    for (FieldSpec& s : f) {
        off = align_up(off, kBlockAlign);
        s.offset = off;
        off += s.bytes();
    }
    block_size = align_up(off, kBlockAlign);
    return f;
}

// Little-endian scalar access into a byte buffer (the format is little-endian and only
// little-endian hosts are supported, so this is a plain copy).
template <typename T>
void put(unsigned char* buf, std::uint64_t off, T v) {
    std::memcpy(buf + off, &v, sizeof(T));
}
template <typename T>
T get(const unsigned char* buf, std::uint64_t off) {
    T v;
    std::memcpy(&v, buf + off, sizeof(T));
    return v;
}

inline bool host_is_little_endian() {
    const std::uint16_t one = 1;
    unsigned char b;
    std::memcpy(&b, &one, 1);
    return b == 1;
}

}  // namespace hp::detail
