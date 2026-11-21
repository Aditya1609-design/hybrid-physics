#include <doctest/doctest.h>

#include <cmath>
#include <cstring>
#include <nlohmann/json.hpp>
#include <stdexcept>
#include <string>

#include "hp/mesh/generators.hpp"
#include "hp/scene/make_state.hpp"
#include "hp/scene/scene.hpp"
#include "hp/solver/xpbd.hpp"

namespace {

using nlohmann::json;

// A 0.5 m cube with 4 cells per side (125 vertices) centred at `center`, no colliders.
json cube_scenario(const hp::Vec3& center) {
    json doc = json::parse(R"({"format_version": 1, "name": "cube", "seed": 0, "frames": 60,
        "bodies": [{"mesh": {"type": "box", "size": [0.5, 0.5, 0.5], "cells": [4, 4, 4]}}]})");
    doc["bodies"][0]["position"] = {center.x(), center.y(), center.z()};
    return doc;
}

hp::Scene build(const json& doc) { return hp::build_scene(hp::parse_scenario(doc.dump())); }

std::string build_error(const json& doc) {
    try {
        build(doc);
    } catch (const std::invalid_argument& e) {
        return e.what();
    }
    return "";
}

bool contains(const std::string& text, const std::string& part) {
    return text.find(part) != std::string::npos;
}

bool any_warning_contains(const hp::Scene& scene, const std::string& part) {
    for (const std::string& w : scene.warnings) {
        if (contains(w, part)) return true;
    }
    return false;
}

template <typename T>
bool same_bytes(const std::vector<T>& a, const std::vector<T>& b) {
    return a.size() == b.size() && std::memcmp(a.data(), b.data(), a.size() * sizeof(T)) == 0;
}

}  // namespace

TEST_CASE("a scenario runs byte-identically to the same scene built in C++") {
    json doc = cube_scenario(hp::Vec3(0, 0.6f, 0));
    doc["bodies"][0]["material"] = {{"density", 800}, {"youngs_modulus", 2e6}, {"damping", 0.1}};
    doc["bodies"][0]["velocity"] = {1.0, 0.0, 0.0};
    doc["colliders"] = json::parse(R"([
        {"type": "plane", "point": [0, 0, 0], "normal": [0, 1, 0], "friction": 0.5},
        {"type": "sphere", "center": [0.8, 0.2, 0], "radius": 0.4, "friction": 0.2}])");
    hp::Scene scene = build(doc);
    CHECK(scene.warnings.empty());

    // The same setup through the existing API.
    hp::BoxParams params;
    params.size = hp::Vec3(0.5f, 0.5f, 0.5f);
    params.cells_x = params.cells_y = params.cells_z = 4;
    params.center = hp::Vec3(0, 0.6f, 0);
    const hp::TetMesh mesh = hp::make_box(params);
    hp::Material material;
    material.density = 800.0f;
    material.edge_compliance = static_cast<float>(1.0 / 2e6);
    material.damping = 0.1f;
    hp::SimState state = hp::make_state(mesh, material.density);
    state.velocities.assign(state.velocities.size(), hp::Vec3(1, 0, 0));
    hp::XpbdSolver solver(
        hp::build_constraints(mesh, hp::build_topology(mesh), material), material,
        hp::XpbdSettings::realtime(),
        {hp::Collider::plane(hp::Vec3::Zero(), hp::Vec3::UnitY(), 0.5f),
         hp::Collider::sphere(hp::Vec3(0.8f, 0.2f, 0), 0.4f, 0.2f)});

    REQUIRE(same_bytes(scene.mesh.rest_positions, mesh.rest_positions));
    const hp::RegionMask mask = hp::RegionMask::all(state.num_vertices());
    for (int f = 0; f < 60; ++f) {
        scene.solver.step(scene.state, mask, hp::kFrameDt);
        solver.step(state, mask, hp::kFrameDt);
    }
    CHECK(same_bytes(scene.state.positions, state.positions));
    CHECK(same_bytes(scene.state.velocities, state.velocities));
    CHECK(same_bytes(scene.state.inv_mass, state.inv_mass));
}

TEST_CASE("rotation and translation keep the rest shape") {
    json doc = cube_scenario(hp::Vec3::Zero());
    doc["bodies"][0]["mesh"] = {{"type", "beam"}, {"length", 1.0}, {"thickness", 0.2},
                                {"cells_along", 10}, {"cells_across", 2}};
    const hp::Scene plain = build(doc);
    doc["bodies"][0]["position"] = {1.0, 2.0, 3.0};
    doc["bodies"][0]["rotation"] = {{"axis", {1, 1, 0}}, {"angle_deg", 30}};
    const hp::Scene moved = build(doc);
    CHECK_NOTHROW(moved.state.validate());
    CHECK_NOTHROW(moved.mesh.validate());

    for (hp::Index t = 0; t < moved.mesh.num_tets(); ++t) {
        CHECK(hp::tet_volume(moved.mesh, t) ==
              doctest::Approx(hp::tet_volume(plain.mesh, t)).epsilon(1e-4));
    }
    Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
    for (const hp::Vec3& p : moved.state.positions) centroid += p.cast<double>();
    centroid /= static_cast<double>(moved.state.num_vertices());
    CHECK((centroid - Eigen::Vector3d(1, 2, 3)).norm() < 1e-5);
    // Positions and rest positions agree, so the body starts at rest shape.
    CHECK(same_bytes(moved.state.positions, moved.state.rest_positions));
    // Edge lengths are unchanged by the rigid motion.
    for (const hp::Edge& e : moved.topology.edges) {
        const auto len = [&](const hp::Scene& s) {
            return (s.state.positions[static_cast<size_t>(e[0])] -
                    s.state.positions[static_cast<size_t>(e[1])])
                .norm();
        };
        CHECK(len(moved) == doctest::Approx(len(plain)).epsilon(1e-5));
    }
}

TEST_CASE("initial velocity is v + w x r about the centre of mass, and pins stay still") {
    json doc = cube_scenario(hp::Vec3(0, 1, 0));
    doc["bodies"][0]["velocity"] = {1.0, 0.0, 0.0};
    doc["bodies"][0]["angular_velocity"] = {0.0, 0.0, 2.0};
    const hp::Scene free_body = build(doc);

    // The uniform cube's centre of mass is its centre, so momentum is m * v.
    const hp::Vec3 center(0, 1, 0);
    const hp::Vec3 w(0, 0, 2);
    Eigen::Vector3d momentum = Eigen::Vector3d::Zero();
    double mass = 0.0;
    for (size_t v = 0; v < free_body.state.positions.size(); ++v) {
        const hp::Vec3 expected =
            hp::Vec3(1, 0, 0) + w.cross(free_body.state.positions[v] - center);
        CHECK((free_body.state.velocities[v] - expected).norm() < 1e-5f);
        const double m = 1.0 / static_cast<double>(free_body.state.inv_mass[v]);
        momentum += m * free_body.state.velocities[v].cast<double>();
        mass += m;
    }
    CHECK((momentum / mass - Eigen::Vector3d(1, 0, 0)).norm() < 1e-5);

    // Pin the top face: 25 vertices, zero velocity and inverse mass.
    doc["bodies"][0]["pins"] = {{{"min", {-1, 1.2, -1}}, {"max", {1, 1.3, 1}}}};
    const hp::Scene pinned = build(doc);
    hp::Index count = 0;
    for (size_t v = 0; v < pinned.state.positions.size(); ++v) {
        if (pinned.state.inv_mass[v] != 0.0f) continue;
        ++count;
        CHECK(pinned.state.positions[v].y() == doctest::Approx(1.25f));
        CHECK(pinned.state.velocities[v] == hp::Vec3::Zero());
    }
    CHECK(count == 25);
}

TEST_CASE("a pin box that selects nothing is an error") {
    json doc = cube_scenario(hp::Vec3::Zero());
    doc["bodies"][0]["pins"] = {{{"min", {5, 5, 5}}, {"max", {6, 6, 6}}}};
    CHECK(contains(build_error(doc), "bodies[0].pins[0]: selects no vertex"));
}

TEST_CASE("a body may touch a collider but not start inside one") {
    const json ground = json::parse(R"([{"type": "plane", "point": [0, 0, 0],
                                         "normal": [0, 1, 0]}])");
    // Bottom face exactly on the plane: signed distance 0 is allowed.
    json touching = cube_scenario(hp::Vec3(0, 0.25f, 0));
    touching["colliders"] = ground;
    CHECK_NOTHROW(build(touching));

    // 5 cm below: the bottom layer of 25 vertices penetrates.
    json inside = cube_scenario(hp::Vec3(0, 0.2f, 0));
    inside["colliders"] = ground;
    const std::string msg = build_error(inside);
    CHECK(contains(msg, "starts inside colliders[0] (plane)"));
    CHECK(contains(msg, "25 vertices"));

    // Pinning the overlapping vertices does not make it acceptable.
    inside["bodies"][0]["pins"] = {{{"min", {-1, -1, -1}}, {"max", {1, -0.01, 1}}}};
    CHECK(contains(build_error(inside), "starts inside colliders[0] (plane)"));

    // A sphere collider overlapping the body's corner.
    json sphere = cube_scenario(hp::Vec3::Zero());
    sphere["colliders"] = {{{"type", "sphere"}, {"center", {0.5, 0.5, 0.5}}, {"radius", 0.5}}};
    CHECK(contains(build_error(sphere), "starts inside colliders[0] (sphere)"));
}

TEST_CASE("small sphere colliders produce a warning") {
    json doc = cube_scenario(hp::Vec3(0, 1, 0));
    doc["colliders"] = {{{"type", "sphere"}, {"center", {0, 0, 0}}, {"radius", 0.05}}};
    const hp::Scene small = build(doc);
    CHECK(any_warning_contains(small, "colliders[0]: sphere radius"));

    doc["colliders"][0]["radius"] = 0.5;
    CHECK(build(doc).warnings.empty());
}

TEST_CASE("colliders a fast vertex could cross produce a warning") {
    // 20 m/s on the real-time preset moves a vertex about 3.4 cm per substep.
    json doc = cube_scenario(hp::Vec3(0, 1, 0));
    doc["frames"] = 1;
    doc["bodies"][0]["velocity"] = {0.0, -20.0, 0.0};
    doc["colliders"] = {{{"type", "box"}, {"center", {0, 0, 0}}, {"half_extents", {1, 0.01, 1}}},
                        {{"type", "plane"}, {"point", {0, -1, 0}}, {"normal", {0, 1, 0}}}};
    const hp::Scene thin = build(doc);
    REQUIRE(thin.warnings.size() == 1);
    CHECK(contains(thin.warnings[0], "colliders[0]: box is thin enough to tunnel"));

    doc["colliders"][0]["half_extents"] = {1, 0.1, 1};
    CHECK(build(doc).warnings.empty());

    // The reference preset's smaller substeps make the thin box safe as well.
    doc["colliders"][0]["half_extents"] = {1, 0.01, 1};
    doc["solver"] = {{"preset", "reference"}};
    CHECK(build(doc).warnings.empty());
}

TEST_CASE("a mesh too large for int32 indices is rejected by build_scene") {
    json doc = cube_scenario(hp::Vec3::Zero());
    doc["bodies"][0]["mesh"]["cells"] = {1300, 1300, 1300};
    CHECK_NOTHROW(hp::parse_scenario(doc.dump()));  // each count alone is a valid int32
    CHECK(contains(build_error(doc), "too many cells"));
}

TEST_CASE("more than one body is rejected for now") {
    json doc = cube_scenario(hp::Vec3::Zero());
    doc["bodies"].push_back(doc["bodies"][0]);
    CHECK(contains(build_error(doc), "exactly one body is supported"));
}
