#include <doctest/doctest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <map>
#include <stdexcept>

#include "hp/mesh/generators.hpp"

namespace {

// Counts how many tets use each triangle face (vertex triple sorted).
std::map<std::array<hp::Index, 3>, int> face_counts(const hp::TetMesh& mesh) {
    std::map<std::array<hp::Index, 3>, int> counts;
    for (const hp::Tet& t : mesh.tets) {
        for (int skip = 0; skip < 4; ++skip) {
            std::array<hp::Index, 3> f{};
            int n = 0;
            for (int a = 0; a < 4; ++a)
                if (a != skip) f[static_cast<size_t>(n++)] = t[static_cast<size_t>(a)];
            std::sort(f.begin(), f.end());
            ++counts[f];
        }
    }
    return counts;
}

// A conforming mesh has every face shared by at most two tets; returns the boundary face count.
int check_conforming(const hp::TetMesh& mesh) {
    int boundary = 0;
    for (const auto& [face, count] : face_counts(mesh)) {
        REQUIRE(count <= 2);
        if (count == 1) ++boundary;
    }
    return boundary;
}

bool same_mesh(const hp::TetMesh& a, const hp::TetMesh& b) {
    return a.rest_positions.size() == b.rest_positions.size() && a.tets == b.tets &&
           std::memcmp(a.rest_positions.data(), b.rest_positions.data(),
                       a.rest_positions.size() * sizeof(hp::Vec3)) == 0;
}

}  // namespace

TEST_CASE("single cube splits into 5 positive tets") {
    hp::BoxParams p;
    p.cells_x = p.cells_y = p.cells_z = 1;
    const hp::TetMesh mesh = hp::make_box(p);
    CHECK(mesh.num_vertices() == 8);
    CHECK(mesh.num_tets() == 5);
    CHECK_NOTHROW(mesh.validate());
    CHECK(hp::total_volume(mesh) == doctest::Approx(1.0).epsilon(1e-6));
    CHECK(check_conforming(mesh) == 12);
}

TEST_CASE("box mesh has expected counts, volume and a conforming boundary") {
    hp::BoxParams p;
    p.size = hp::Vec3(2.0f, 1.0f, 0.5f);
    p.cells_x = 6;
    p.cells_y = 4;
    p.cells_z = 3;
    p.center = hp::Vec3(1.0f, 2.0f, 3.0f);
    const hp::TetMesh mesh = hp::make_box(p);

    CHECK(mesh.num_vertices() == 7 * 5 * 4);
    CHECK(mesh.num_tets() == 5 * 6 * 4 * 3);
    CHECK_NOTHROW(mesh.validate());
    CHECK(hp::total_volume(mesh) == doctest::Approx(1.0).epsilon(1e-5));
    // Two triangles per boundary cell face.
    CHECK(check_conforming(mesh) == 2 * 2 * (6 * 4 + 4 * 3 + 6 * 3));

    hp::Vec3 lo = mesh.rest_positions[0], hi = mesh.rest_positions[0];
    for (const hp::Vec3& x : mesh.rest_positions) {
        lo = lo.cwiseMin(x);
        hi = hi.cwiseMax(x);
    }
    CHECK(lo.isApprox(hp::Vec3(0.0f, 1.5f, 2.75f)));
    CHECK(hi.isApprox(hp::Vec3(2.0f, 2.5f, 3.25f)));
}

TEST_CASE("default box and beam have the planned sizes") {
    const hp::TetMesh box = hp::make_box({});
    CHECK(box.num_vertices() == 729);
    CHECK(box.num_tets() == 2560);
    CHECK_NOTHROW(box.validate());

    const hp::TetMesh beam = hp::make_beam({});
    CHECK(beam.num_vertices() == 756);
    CHECK(beam.num_tets() == 2500);
    CHECK_NOTHROW(beam.validate());
    CHECK(hp::total_volume(beam) == doctest::Approx(2.0 * 0.5 * 0.5).epsilon(1e-5));
}

TEST_CASE("sphere mesh is valid, conforming, symmetric and converges in volume") {
    const hp::TetMesh coarse = hp::make_sphere({});
    CHECK_NOTHROW(coarse.validate());
    check_conforming(coarse);
    MESSAGE("default sphere: " << coarse.num_vertices() << " vertices, " << coarse.num_tets()
                               << " tets");
    CHECK(coarse.num_vertices() == 823);
    CHECK(coarse.num_tets() == 2760);

    // Mirror symmetry about the centre: the vertex centroid sits at the centre.
    hp::SphereParams p;
    p.center = hp::Vec3(0.25f, -1.0f, 2.0f);
    const hp::TetMesh shifted = hp::make_sphere(p);
    hp::Vec3 centroid = hp::Vec3::Zero();
    for (const hp::Vec3& x : shifted.rest_positions) centroid += x;
    centroid /= static_cast<float>(shifted.num_vertices());
    CHECK((centroid - p.center).norm() < 1e-5f);

    const double exact = 4.0 / 3.0 * 3.14159265358979 * 0.125;
    p = {};
    p.cells_per_diameter = 10;
    const double err10 = std::abs(hp::total_volume(hp::make_sphere(p)) - exact) / exact;
    p.cells_per_diameter = 32;
    const hp::TetMesh fine = hp::make_sphere(p);
    CHECK_NOTHROW(fine.validate());
    const double err32 = std::abs(hp::total_volume(fine) - exact) / exact;
    MESSAGE("sphere volume error: " << err10 << " at 10 cells, " << err32 << " at 32 cells");
    CHECK(err10 < 0.08);
    CHECK(err32 < 0.015);
}

TEST_CASE("generators are deterministic") {
    CHECK(same_mesh(hp::make_box({}), hp::make_box({})));
    CHECK(same_mesh(hp::make_beam({}), hp::make_beam({})));
    CHECK(same_mesh(hp::make_sphere({}), hp::make_sphere({})));
}

TEST_CASE("generators reject invalid parameters") {
    hp::BoxParams p;
    p.cells_y = 0;
    CHECK_THROWS_AS(hp::make_box(p), std::invalid_argument);
    p = {};
    p.size.z() = -1.0f;
    CHECK_THROWS_AS(hp::make_box(p), std::invalid_argument);
    hp::BeamParams b;
    b.thickness = 0.0f;
    CHECK_THROWS_AS(hp::make_beam(b), std::invalid_argument);
    hp::SphereParams sp;
    sp.cells_per_diameter = 0;
    CHECK_THROWS_AS(hp::make_sphere(sp), std::invalid_argument);
}

TEST_CASE("validate catches broken meshes") {
    hp::BoxParams p;
    p.cells_x = p.cells_y = p.cells_z = 1;
    hp::TetMesh mesh = hp::make_box(p);

    hp::TetMesh flipped = mesh;
    std::swap(flipped.tets[0][0], flipped.tets[0][1]);
    CHECK_THROWS_AS(flipped.validate(), std::invalid_argument);

    hp::TetMesh out_of_range = mesh;
    out_of_range.tets[0][0] = 8;
    CHECK_THROWS_AS(out_of_range.validate(), std::invalid_argument);

    hp::TetMesh unused = mesh;
    unused.rest_positions.push_back(hp::Vec3::Zero());
    CHECK_THROWS_AS(unused.validate(), std::invalid_argument);
}
