#include "core/colors.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

namespace hp::viewer {

namespace {

// Distinct, mid-brightness colors that stay readable under shading.
constexpr std::array<Rgb, 12> kPalette = {{
    {76, 120, 168},
    {245, 133, 24},
    {84, 162, 75},
    {228, 87, 86},
    {114, 183, 178},
    {238, 202, 59},
    {178, 121, 162},
    {255, 157, 166},
    {157, 117, 93},
    {186, 176, 172},
    {139, 196, 233},
    {150, 200, 110},
}};

std::uint8_t to_byte(float v) {
    return static_cast<std::uint8_t>(std::clamp(std::lround(v), 0L, 255L));
}

}  // namespace

const char* color_mode_name(ColorMode mode) {
    switch (mode) {
        case ColorMode::Owner: return "solver owner";
        case ColorMode::Region: return "region";
        case ColorMode::Body: return "body";
        case ColorMode::Plain: return "plain";
    }
    return "?";
}

ColorMode next_color_mode(ColorMode mode) {
    switch (mode) {
        case ColorMode::Owner: return ColorMode::Region;
        case ColorMode::Region: return ColorMode::Body;
        case ColorMode::Body: return ColorMode::Plain;
        case ColorMode::Plain: return ColorMode::Owner;
    }
    return ColorMode::Owner;
}

Rgb owner_color(SolverId owner) {
    switch (owner) {
        case SolverId::Xpbd: return {70, 130, 220};
        case SolverId::Gnn: return {235, 140, 50};
    }
    return {128, 128, 128};
}

Rgb palette_color(Index id) {
    if (id < 0) return {128, 128, 128};
    return kPalette[static_cast<size_t>(id) % kPalette.size()];
}

Rgb plain_color() { return {180, 186, 196}; }

void vertex_colors(ColorMode mode, const std::vector<Index>& region_id,
                   const std::vector<SolverId>& solver_owner, const std::vector<Index>& body_id,
                   std::vector<Rgb>& out) {
    const size_t n = region_id.size();
    out.resize(n);
    for (size_t v = 0; v < n; ++v) {
        switch (mode) {
            case ColorMode::Owner: out[v] = owner_color(solver_owner[v]); break;
            case ColorMode::Region: out[v] = palette_color(region_id[v]); break;
            case ColorMode::Body: out[v] = palette_color(body_id[v]); break;
            case ColorMode::Plain: out[v] = plain_color(); break;
        }
    }
}

Rgb shade(Rgb base, const Vec3& normal, const Vec3& key_dir, const Vec3& view_dir) {
    const float key = std::max(0.0f, normal.dot(key_dir));
    const float head = std::max(0.0f, normal.dot(view_dir));
    const float k = 0.28f + 0.52f * key + 0.25f * head;
    return {to_byte(k * static_cast<float>(base.r)), to_byte(k * static_cast<float>(base.g)),
            to_byte(k * static_cast<float>(base.b))};
}

OwnerPreview make_owner_preview(const std::vector<Vec3>& rest_positions, Index slabs) {
    OwnerPreview p;
    const size_t n = rest_positions.size();
    p.region_id.assign(n, 0);
    p.solver_owner.assign(n, SolverId::Xpbd);
    if (n == 0 || slabs < 1) return p;

    float lo = std::numeric_limits<float>::max();
    float hi = std::numeric_limits<float>::lowest();
    for (const Vec3& x : rest_positions) {
        lo = std::min(lo, x.x());
        hi = std::max(hi, x.x());
    }
    const float width = hi - lo;
    for (size_t v = 0; v < n; ++v) {
        const float t = width > 0.0f ? (rest_positions[v].x() - lo) / width : 0.0f;
        const Index s =
            std::min(static_cast<Index>(t * static_cast<float>(slabs)), static_cast<Index>(slabs - 1));
        p.region_id[v] = s;
        p.solver_owner[v] = (s % 2 == 0) ? SolverId::Xpbd : SolverId::Gnn;
    }
    return p;
}

}  // namespace hp::viewer
