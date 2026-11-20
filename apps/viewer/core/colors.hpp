#pragma once

#include <cstdint>
#include <vector>

#include "hp/core/types.hpp"

namespace hp::viewer {

struct Rgb {
    std::uint8_t r = 0;
    std::uint8_t g = 0;
    std::uint8_t b = 0;
};

inline bool operator==(const Rgb& a, const Rgb& b) {
    return a.r == b.r && a.g == b.g && a.b == b.b;
}
inline bool operator!=(const Rgb& a, const Rgb& b) { return !(a == b); }

// What the surface color shows. Owner and region read SimState's per-vertex arrays directly.
enum class ColorMode : std::uint8_t { Owner, Region, Body, Plain };

const char* color_mode_name(ColorMode mode);
ColorMode next_color_mode(ColorMode mode);

// Fixed colors per solver: XPBD blue, GNN orange.
Rgb owner_color(SolverId owner);
// A fixed 12-color palette indexed by id modulo 12; negative ids are grey.
Rgb palette_color(Index id);
// Neutral surface color for ColorMode::Plain.
Rgb plain_color();

// One color per vertex for `mode`. The arrays must all have the same length.
void vertex_colors(ColorMode mode, const std::vector<Index>& region_id,
                   const std::vector<SolverId>& solver_owner, const std::vector<Index>& body_id,
                   std::vector<Rgb>& out);

// Lambert shading: ambient plus a key light and a light from the viewer. `normal`,
// `key_dir` (towards the light) and `view_dir` (towards the camera) must be unit vectors.
Rgb shade(Rgb base, const Vec3& normal, const Vec3& key_dir, const Vec3& view_dir);

// Display-only stand-in for region and owner arrays, so the coloring can be checked before
// regions exist: `slabs` equal slabs along x of the rest shape, owners alternating per slab.
// The simulation state is never changed.
struct OwnerPreview {
    std::vector<Index> region_id;
    std::vector<SolverId> solver_owner;
};
OwnerPreview make_owner_preview(const std::vector<Vec3>& rest_positions, Index slabs);

}  // namespace hp::viewer
