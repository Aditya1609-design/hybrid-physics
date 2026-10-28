#include <doctest/doctest.h>

#include <cmath>
#include <nlohmann/json.hpp>
#include <stdexcept>
#include <string>
#include <variant>

#include "hp/scene/scenario.hpp"

namespace {

using nlohmann::json;

// A scenario that uses every key, so tests can break one field at a time.
json full_scenario() {
    return json::parse(R"({
        "format_version": 1,
        "name": "everything",
        "seed": 42,
        "frames": 120,
        "solver": {"preset": "reference", "substeps": 64, "iterations": 2,
                   "gravity": [0, -5, 0]},
        "bodies": [{
            "mesh": {"type": "box", "size": [0.5, 0.25, 1.0], "cells": [4, 2, 8]},
            "material": {"density": 800, "youngs_modulus": 2e6, "bulk_modulus": 4e7,
                         "damping": 0.5},
            "position": [1, 2, 3],
            "rotation": {"axis": [0, 0, 2], "angle_deg": 90},
            "velocity": [0.5, 0, 0],
            "angular_velocity": [0, 1, 0],
            "pins": [{"min": [-1, -1, -1], "max": [1, 1, 1]}]
        }],
        "colliders": [
            {"type": "plane", "point": [0, 0, 0], "normal": [0, 2, 0], "friction": 0.5},
            {"type": "sphere", "center": [1, 0, 0], "radius": 0.3},
            {"type": "box", "center": [0, 0, 1], "half_extents": [0.1, 0.2, 0.3],
             "rotation": {"axis": [0, 1, 0], "angle_deg": 30}, "friction": 0.25}
        ]
    })");
}

json minimal_scenario() {
    return json::parse(R"({"format_version": 1, "name": "minimal", "seed": 0, "frames": 1,
                           "bodies": [{"mesh": {"type": "box"}}]})");
}

// The message parse_scenario throws for `doc`, or "" if it parses.
std::string error_of(const json& doc) {
    try {
        hp::parse_scenario(doc.dump());
    } catch (const std::invalid_argument& e) {
        return e.what();
    }
    return "";
}

bool contains(const std::string& text, const std::string& part) {
    return text.find(part) != std::string::npos;
}

}  // namespace

TEST_CASE("parse_scenario reads every field") {
    const hp::Scenario s = hp::parse_scenario(full_scenario().dump());
    CHECK(s.name == "everything");
    CHECK(s.seed == 42);
    CHECK(s.frames == 120);
    CHECK(s.preset == "reference");
    CHECK(s.solver.substeps == 64);
    CHECK(s.solver.iterations == 2);
    CHECK(s.solver.gravity == hp::Vec3(0, -5, 0));

    REQUIRE(s.bodies.size() == 1);
    const hp::ScenarioBody& b = s.bodies[0];
    REQUIRE(std::holds_alternative<hp::BoxParams>(b.mesh));
    const auto& box = std::get<hp::BoxParams>(b.mesh);
    CHECK(box.size == hp::Vec3(0.5f, 0.25f, 1.0f));
    CHECK(box.cells_x == 4);
    CHECK(box.cells_y == 2);
    CHECK(box.cells_z == 8);
    CHECK(b.material.density == 800.0f);
    CHECK(b.material.edge_compliance == doctest::Approx(1.0 / 2e6).epsilon(1e-6));
    CHECK(b.material.volume_compliance == doctest::Approx(1.0 / 4e7).epsilon(1e-6));
    CHECK(b.material.damping == 0.5f);
    CHECK(b.position == hp::Vec3(1, 2, 3));
    // 90 degrees about +z (axis given unnormalized) maps x to y.
    CHECK((b.rotation * hp::Vec3::UnitX() - hp::Vec3::UnitY()).norm() < 1e-6f);
    CHECK(b.velocity == hp::Vec3(0.5f, 0, 0));
    CHECK(b.angular_velocity == hp::Vec3(0, 1, 0));
    REQUIRE(b.pins.size() == 1);
    CHECK(b.pins[0].min == hp::Vec3(-1, -1, -1));
    CHECK(b.pins[0].max == hp::Vec3(1, 1, 1));

    REQUIRE(s.colliders.size() == 3);
    CHECK(s.colliders[0].kind == hp::ColliderKind::Plane);
    CHECK(s.colliders[0].normal == hp::Vec3::UnitY());
    CHECK(s.colliders[0].friction == 0.5f);
    CHECK(s.colliders[1].kind == hp::ColliderKind::Sphere);
    CHECK(s.colliders[1].radius == 0.3f);
    CHECK(s.colliders[1].friction == 0.0f);
    CHECK(s.colliders[2].kind == hp::ColliderKind::Box);
    CHECK(s.colliders[2].half_extents == hp::Vec3(0.1f, 0.2f, 0.3f));
    CHECK(s.colliders[2].rotation(0, 0) == doctest::Approx(std::cos(30.0 * M_PI / 180.0)));
    CHECK_NOTHROW(s.colliders[2].validate());
}

TEST_CASE("parse_scenario fills documented defaults") {
    const hp::Scenario s = hp::parse_scenario(minimal_scenario().dump());
    const hp::XpbdSettings rt = hp::XpbdSettings::realtime();
    CHECK(s.preset == "realtime");
    CHECK(s.solver.substeps == rt.substeps);
    CHECK(s.solver.iterations == rt.iterations);
    CHECK(s.solver.gravity == rt.gravity);
    CHECK(s.colliders.empty());

    REQUIRE(s.bodies.size() == 1);
    const hp::ScenarioBody& b = s.bodies[0];
    const hp::BoxParams box_default;
    const auto& box = std::get<hp::BoxParams>(b.mesh);
    CHECK(box.size == box_default.size);
    CHECK(box.cells_x == box_default.cells_x);
    // Absent material keys keep the Material defaults bit for bit.
    const hp::Material m;
    CHECK(b.material.density == m.density);
    CHECK(b.material.edge_compliance == m.edge_compliance);
    CHECK(b.material.volume_compliance == m.volume_compliance);
    CHECK(b.material.damping == m.damping);
    CHECK(b.position == hp::Vec3::Zero());
    CHECK(b.rotation == hp::Mat3::Identity());
    CHECK(b.velocity == hp::Vec3::Zero());
    CHECK(b.angular_velocity == hp::Vec3::Zero());
    CHECK(b.pins.empty());
}

TEST_CASE("parse_scenario reads beam and sphere meshes") {
    json doc = minimal_scenario();
    doc["bodies"][0]["mesh"] = {{"type", "beam"}, {"length", 3.0}, {"thickness", 0.2},
                                {"cells_along", 30}, {"cells_across", 2}};
    const auto beam = std::get<hp::BeamParams>(hp::parse_scenario(doc.dump()).bodies[0].mesh);
    CHECK(beam.length == 3.0f);
    CHECK(beam.thickness == 0.2f);
    CHECK(beam.cells_along == 30);
    CHECK(beam.cells_across == 2);

    doc["bodies"][0]["mesh"] = {{"type", "sphere"}, {"radius", 0.4}, {"cells_per_diameter", 7}};
    const auto sphere =
        std::get<hp::SphereParams>(hp::parse_scenario(doc.dump()).bodies[0].mesh);
    CHECK(sphere.radius == 0.4f);
    CHECK(sphere.cells_per_diameter == 7);
}

TEST_CASE("parse_scenario reports bad input with the key's path") {
    struct Case {
        const char* what;
        void (*edit)(json&);
        const char* expected;
    };
    const Case cases[] = {
        {"missing key", [](json& j) { j.erase("seed"); }, "seed: is required"},
        {"unknown root key", [](json& j) { j["sed"] = 1; }, "sed: unknown key"},
        {"misspelt material key",
         [](json& j) { j["bodies"][0]["material"]["youngs_modulos"] = 1e6; },
         "bodies[0].material.youngs_modulos: unknown key"},
        {"key of another collider type",
         [](json& j) { j["colliders"][0]["radius"] = 1.0; }, "colliders[0].radius: unknown key"},
        {"wrong type", [](json& j) { j["frames"] = "10"; }, "frames: must be an integer"},
        {"fractional count", [](json& j) { j["frames"] = 1.5; }, "frames: must be an integer"},
        {"zero frames", [](json& j) { j["frames"] = 0; }, "frames: must be at least 1"},
        {"future version", [](json& j) { j["format_version"] = 2; },
         "format_version: unsupported version 2 (expected 1)"},
        {"empty name", [](json& j) { j["name"] = ""; }, "name: must not be empty"},
        {"negative seed", [](json& j) { j["seed"] = -1; }, "seed: must be a non-negative integer"},
        {"bad preset", [](json& j) { j["solver"]["preset"] = "fast"; },
         "solver.preset: must be \"realtime\" or \"reference\""},
        {"zero substeps", [](json& j) { j["solver"]["substeps"] = 0; },
         "solver.substeps: must be at least 1"},
        {"no bodies", [](json& j) { j["bodies"] = json::array(); },
         "bodies: must contain at least one body"},
        {"bad mesh type", [](json& j) { j["bodies"][0]["mesh"]["type"] = "torus"; },
         "bodies[0].mesh.type: must be"},
        {"zero cells", [](json& j) { j["bodies"][0]["mesh"]["cells"][1] = 0; },
         "bodies[0].mesh.cells[1]: must be at least 1"},
        {"negative size", [](json& j) { j["bodies"][0]["mesh"]["size"][2] = -1.0; },
         "bodies[0].mesh.size: must have positive components"},
        {"negative modulus", [](json& j) { j["bodies"][0]["material"]["youngs_modulus"] = -1; },
         "bodies[0].material.youngs_modulus: must be positive"},
        {"negative damping", [](json& j) { j["bodies"][0]["material"]["damping"] = -1; },
         "bodies[0].material:"},
        {"short vector", [](json& j) { j["bodies"][0]["position"] = {1, 2}; },
         "bodies[0].position: must be an array of 3 numbers"},
        {"non-number in vector", [](json& j) { j["bodies"][0]["velocity"][1] = "x"; },
         "bodies[0].velocity[1]: must be a number"},
        {"beyond float range", [](json& j) { j["bodies"][0]["position"][0] = 1e40; },
         "bodies[0].position[0]: must be a finite float32 number"},
        {"zero rotation axis",
         [](json& j) { j["bodies"][0]["rotation"]["axis"] = {0, 0, 0}; },
         "bodies[0].rotation.axis: must be a non-zero vector"},
        {"inverted pin box", [](json& j) { j["bodies"][0]["pins"][0]["min"][0] = 2; },
         "bodies[0].pins[0]: min must not exceed max"},
        {"bad collider type", [](json& j) { j["colliders"][1]["type"] = "cone"; },
         "colliders[1].type: must be"},
        {"negative radius", [](json& j) { j["colliders"][1]["radius"] = -0.3; },
         "colliders[1].radius: must be positive"},
        {"negative friction", [](json& j) { j["colliders"][2]["friction"] = -0.1; },
         "colliders[2].friction: must not be negative"},
        {"zero plane normal", [](json& j) { j["colliders"][0]["normal"] = {0, 0, 0}; },
         "colliders[0].normal: must be non-zero"},
        {"colliders not a list", [](json& j) { j["colliders"] = json::object(); },
         "colliders: must be an array"},
        {"body not an object", [](json& j) { j["bodies"][0] = 3; },
         "bodies[0]: must be an object"},
    };
    for (const Case& c : cases) {
        CAPTURE(c.what);
        json doc = full_scenario();
        c.edit(doc);
        const std::string msg = error_of(doc);
        CAPTURE(msg);
        CHECK(contains(msg, c.expected));
    }
}

TEST_CASE("parse_scenario rejects malformed JSON and non-object documents") {
    CHECK_THROWS_AS(hp::parse_scenario("{\"name\": "), std::invalid_argument);
    try {
        hp::parse_scenario("{\"name\": ");
    } catch (const std::invalid_argument& e) {
        CHECK(contains(e.what(), "invalid JSON"));
    }
    CHECK(contains(error_of(json::array()), "(root): must be an object"));
}

TEST_CASE("load_scenario reports a missing file") {
    CHECK_THROWS_AS(hp::load_scenario("no/such/scenario.json"), std::runtime_error);
}
