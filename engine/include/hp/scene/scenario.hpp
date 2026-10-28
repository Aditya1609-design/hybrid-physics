#pragma once

#include <cstdint>
#include <string>
#include <variant>
#include <vector>

#include "hp/collision/collider.hpp"
#include "hp/core/types.hpp"
#include "hp/mesh/generators.hpp"
#include "hp/solver/xpbd.hpp"
#include "hp/solver/xpbd_constraints.hpp"

namespace hp {

// The scenario file version this loader reads.
constexpr int kScenarioFormatVersion = 1;

// Which procedural mesh a body uses. The params' `center` is ignored; the body's pose sets it.
using MeshSpec = std::variant<BoxParams, BeamParams, SphereParams>;

// World-space axis-aligned box: every vertex inside it (initial pose, bounds included) is pinned.
struct PinBox {
    Vec3 min = Vec3::Zero();
    Vec3 max = Vec3::Zero();
};

// One deformable body and its initial conditions.
struct ScenarioBody {
    MeshSpec mesh = BoxParams{};
    Material material;
    Vec3 position = Vec3::Zero();       // where the mesh centre is placed
    Mat3 rotation = Mat3::Identity();   // applied about the mesh centre before moving it
    Vec3 velocity = Vec3::Zero();       // m/s, every vertex
    Vec3 angular_velocity = Vec3::Zero();  // rad/s, about the centre of mass
    std::vector<PinBox> pins;
};

// Everything needed to set up and run one simulation, as read from a scenario file.
struct Scenario {
    std::string name;
    std::uint64_t seed = 0;  // kept with the scenario; nothing random uses it yet
    Index frames = 0;        // frames of kFrameDt to run
    std::string preset;      // "realtime" or "reference", before any overrides
    XpbdSettings solver;
    std::vector<ScenarioBody> bodies;
    std::vector<Collider> colliders;
};

// Parses a scenario from JSON text (format described in docs/scenario_format.md).
// Throws std::invalid_argument for invalid JSON, missing, unknown or mistyped keys and bad
// values; the message names the offending key, e.g. "colliders[1].radius: must be positive".
Scenario parse_scenario(const std::string& json_text);

// Reads and parses a scenario file. Throws std::runtime_error if the file cannot be read,
// and std::invalid_argument as parse_scenario does.
Scenario load_scenario(const std::string& path);

}  // namespace hp
