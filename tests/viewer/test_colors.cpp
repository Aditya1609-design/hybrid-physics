#include <doctest/doctest.h>

#include <set>
#include <tuple>

#include "core/colors.hpp"
#include "hp/mesh/generators.hpp"

using hp::SolverId;
using hp::Vec3;
using namespace hp::viewer;

TEST_CASE("solver owners have distinct fixed colors") {
    CHECK(owner_color(SolverId::Xpbd) != owner_color(SolverId::Gnn));
    CHECK(owner_color(SolverId::Xpbd) == owner_color(SolverId::Xpbd));
}

TEST_CASE("palette colors are deterministic, repeat every 12 ids and grey for negative ids") {
    std::set<std::tuple<int, int, int>> distinct;
    for (hp::Index id = 0; id < 12; ++id) {
        const Rgb c = palette_color(id);
        distinct.insert({c.r, c.g, c.b});
        CHECK(palette_color(id + 12) == c);
        CHECK(palette_color(id + 1200) == c);
    }
    CHECK(distinct.size() == 12);
    CHECK(palette_color(-1) == Rgb{128, 128, 128});
    CHECK(palette_color(-1000) == Rgb{128, 128, 128});
}

TEST_CASE("vertex colors follow the chosen per-vertex array") {
    const std::vector<hp::Index> region = {0, 3, 3, 15};
    const std::vector<SolverId> owner = {SolverId::Xpbd, SolverId::Gnn, SolverId::Xpbd,
                                         SolverId::Gnn};
    const std::vector<hp::Index> body = {0, 0, 1, 1};
    std::vector<Rgb> out;

    vertex_colors(ColorMode::Owner, region, owner, body, out);
    REQUIRE(out.size() == 4);
    for (size_t v = 0; v < 4; ++v) CHECK(out[v] == owner_color(owner[v]));

    vertex_colors(ColorMode::Region, region, owner, body, out);
    for (size_t v = 0; v < 4; ++v) CHECK(out[v] == palette_color(region[v]));

    vertex_colors(ColorMode::Body, region, owner, body, out);
    for (size_t v = 0; v < 4; ++v) CHECK(out[v] == palette_color(body[v]));

    vertex_colors(ColorMode::Plain, region, owner, body, out);
    for (size_t v = 0; v < 4; ++v) CHECK(out[v] == plain_color());
}

TEST_CASE("color modes cycle through all four and back") {
    ColorMode m = ColorMode::Owner;
    std::set<ColorMode> seen;
    for (int i = 0; i < 4; ++i) {
        seen.insert(m);
        m = next_color_mode(m);
    }
    CHECK(seen.size() == 4);
    CHECK(m == ColorMode::Owner);
}

TEST_CASE("shading is brighter towards the light and never overflows") {
    const Vec3 key = Vec3(0.3f, 1.0f, 0.2f).normalized();
    const Vec3 view = Vec3(0.0f, 0.0f, 1.0f);
    const Rgb base{200, 100, 50};
    const Rgb lit = shade(base, key, key, view);
    const Rgb dark = shade(base, -key, key, view);
    CHECK(lit.r > dark.r);
    CHECK(lit.g > dark.g);
    CHECK(dark.r > 0);  // ambient keeps back faces visible

    const Rgb white = shade(Rgb{255, 255, 255}, view, view, view);
    CHECK(white == Rgb{255, 255, 255});
}

TEST_CASE("owner preview splits the rest shape into alternating slabs along x") {
    hp::BeamParams params;
    params.length = 2.0f;
    params.thickness = 0.2f;
    params.cells_along = 8;
    params.cells_across = 2;
    const hp::TetMesh mesh = hp::make_beam(params);
    const OwnerPreview p = make_owner_preview(mesh.rest_positions, 4);
    REQUIRE(p.region_id.size() == mesh.rest_positions.size());
    REQUIRE(p.solver_owner.size() == mesh.rest_positions.size());

    std::set<hp::Index> slabs;
    for (size_t v = 0; v < p.region_id.size(); ++v) {
        const hp::Index s = p.region_id[v];
        CHECK(s >= 0);
        CHECK(s < 4);
        slabs.insert(s);
        CHECK(p.solver_owner[v] == (s % 2 == 0 ? SolverId::Xpbd : SolverId::Gnn));
        // Slab index never decreases with x.
        for (size_t w = 0; w < p.region_id.size(); ++w) {
            if (mesh.rest_positions[w].x() > mesh.rest_positions[v].x()) {
                CHECK(p.region_id[w] >= s);
            }
        }
    }
    CHECK(slabs.size() == 4);

    // A flat shape or a zero slab count falls back to one XPBD region.
    const OwnerPreview flat = make_owner_preview({Vec3(1, 0, 0), Vec3(1, 1, 0)}, 4);
    CHECK(flat.region_id == std::vector<hp::Index>{0, 0});
    const OwnerPreview none = make_owner_preview(mesh.rest_positions, 0);
    for (SolverId o : none.solver_owner) CHECK(o == SolverId::Xpbd);
}
