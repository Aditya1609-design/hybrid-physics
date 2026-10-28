#include "hp/scene/scenario.hpp"

#include <cmath>
#include <fstream>
#include <limits>
#include <nlohmann/json.hpp>
#include <set>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace hp {

namespace {

using nlohmann::json;

[[noreturn]] void fail(const std::string& path, const std::string& what) {
    throw std::invalid_argument("scenario: " + path + ": " + what);
}

// One JSON object being read. Remembers which keys were asked for, so finish() can reject
// the others: an unknown key is almost always a typo that would otherwise be ignored.
class ObjectReader {
public:
    ObjectReader(const json& j, std::string path) : j_(j), path_(std::move(path)) {
        if (!j_.is_object()) fail(path_.empty() ? "(root)" : path_, "must be an object");
    }

    // Path of a key in this object, for error messages.
    std::string at(const std::string& key) const {
        return path_.empty() ? key : path_ + "." + key;
    }

    // The value of `key`, or nullptr if it is absent.
    const json* find(const std::string& key) {
        known_.insert(key);
        const auto it = j_.find(key);
        return it == j_.end() ? nullptr : &*it;
    }

    const json& require(const std::string& key) {
        const json* v = find(key);
        if (v == nullptr) fail(at(key), "is required");
        return *v;
    }

    void finish() const {
        for (auto it = j_.begin(); it != j_.end(); ++it) {
            if (known_.count(it.key()) == 0) fail(at(it.key()), "unknown key");
        }
    }

private:
    const json& j_;
    std::string path_;
    std::set<std::string> known_;
};

Real to_real(const json& v, const std::string& path) {
    if (!v.is_number()) fail(path, "must be a number");
    const double d = v.get<double>();
    if (!std::isfinite(d) || std::abs(d) > static_cast<double>(std::numeric_limits<Real>::max())) {
        fail(path, "must be a finite float32 number");
    }
    return static_cast<Real>(d);
}

Real to_positive_real(const json& v, const std::string& path) {
    const Real r = to_real(v, path);
    if (!(r > 0.0f)) fail(path, "must be positive");
    return r;
}

// An integer that fits in Index. JSON reads non-negative integers as unsigned.
Index to_index(const json& v, const std::string& path) {
    if (!v.is_number_integer()) fail(path, "must be an integer");
    constexpr auto lo = std::numeric_limits<Index>::min();
    constexpr auto hi = std::numeric_limits<Index>::max();
    if (v.is_number_unsigned()) {
        if (v.get<std::uint64_t>() > static_cast<std::uint64_t>(hi)) fail(path, "is too large");
        return static_cast<Index>(v.get<std::uint64_t>());
    }
    const std::int64_t i = v.get<std::int64_t>();
    if (i < lo || i > hi) fail(path, "is out of range");
    return static_cast<Index>(i);
}

Index to_count(const json& v, const std::string& path) {
    const Index i = to_index(v, path);
    if (i < 1) fail(path, "must be at least 1");
    return i;
}

std::string to_string(const json& v, const std::string& path) {
    if (!v.is_string()) fail(path, "must be a string");
    return v.get<std::string>();
}

Vec3 to_vec3(const json& v, const std::string& path) {
    if (!v.is_array() || v.size() != 3) fail(path, "must be an array of 3 numbers");
    Vec3 out;
    for (int k = 0; k < 3; ++k) {
        out[k] = to_real(v[static_cast<size_t>(k)], path + "[" + std::to_string(k) + "]");
    }
    return out;
}

Vec3 to_positive_vec3(const json& v, const std::string& path) {
    const Vec3 out = to_vec3(v, path);
    if (!(out.minCoeff() > 0.0f)) fail(path, "must have positive components");
    return out;
}

// {"axis": [x, y, z], "angle_deg": a}. Built in double so the float matrix is orthonormal to
// rounding, well inside Collider::validate's tolerance.
Mat3 to_rotation(const json& v, const std::string& path) {
    ObjectReader r(v, path);
    const Vec3 axis = to_vec3(r.require("axis"), r.at("axis"));
    const Real angle_deg = to_real(r.require("angle_deg"), r.at("angle_deg"));
    r.finish();
    const double len = axis.cast<double>().norm();
    if (!(len > 0.0) || !std::isfinite(len)) fail(r.at("axis"), "must be a non-zero vector");
    if (angle_deg == 0.0f) return Mat3::Identity();
    const double angle = static_cast<double>(angle_deg) * 3.14159265358979323846 / 180.0;
    const Eigen::AngleAxisd aa(angle, axis.cast<double>() / len);
    return aa.toRotationMatrix().cast<Real>();
}

// Mesh parameters; every key is optional and defaults to the generator's default.
MeshSpec parse_mesh(const json& v, const std::string& path) {
    ObjectReader r(v, path);
    const std::string type = to_string(r.require("type"), r.at("type"));
    MeshSpec spec;
    if (type == "box") {
        BoxParams p;
        if (const json* s = r.find("size")) p.size = to_positive_vec3(*s, r.at("size"));
        if (const json* c = r.find("cells")) {
            const std::string cpath = r.at("cells");
            if (!c->is_array() || c->size() != 3) fail(cpath, "must be an array of 3 integers");
            p.cells_x = to_count((*c)[size_t{0}], cpath + "[0]");
            p.cells_y = to_count((*c)[size_t{1}], cpath + "[1]");
            p.cells_z = to_count((*c)[size_t{2}], cpath + "[2]");
        }
        spec = p;
    } else if (type == "beam") {
        BeamParams p;
        if (const json* x = r.find("length")) p.length = to_positive_real(*x, r.at("length"));
        if (const json* x = r.find("thickness")) {
            p.thickness = to_positive_real(*x, r.at("thickness"));
        }
        if (const json* x = r.find("cells_along")) p.cells_along = to_count(*x, r.at("cells_along"));
        if (const json* x = r.find("cells_across")) {
            p.cells_across = to_count(*x, r.at("cells_across"));
        }
        spec = p;
    } else if (type == "sphere") {
        SphereParams p;
        if (const json* x = r.find("radius")) p.radius = to_positive_real(*x, r.at("radius"));
        if (const json* x = r.find("cells_per_diameter")) {
            p.cells_per_diameter = to_count(*x, r.at("cells_per_diameter"));
        }
        spec = p;
    } else {
        fail(r.at("type"), "must be \"box\", \"beam\" or \"sphere\"");
    }
    r.finish();
    return spec;
}

// Moduli in Pa become compliances (ADR 0005: Y = 1/edge_compliance, K = 1/volume_compliance).
// Absent keys keep the Material defaults exactly.
Material parse_material(const json& v, const std::string& path) {
    ObjectReader r(v, path);
    Material m;
    if (const json* x = r.find("density")) m.density = to_positive_real(*x, r.at("density"));
    if (const json* x = r.find("youngs_modulus")) {
        const Real y = to_positive_real(*x, r.at("youngs_modulus"));
        m.edge_compliance = static_cast<Real>(1.0 / static_cast<double>(y));
    }
    if (const json* x = r.find("bulk_modulus")) {
        const Real k = to_positive_real(*x, r.at("bulk_modulus"));
        m.volume_compliance = static_cast<Real>(1.0 / static_cast<double>(k));
    }
    if (const json* x = r.find("damping")) m.damping = to_real(*x, r.at("damping"));
    r.finish();
    try {
        m.validate();
    } catch (const std::invalid_argument& e) {
        fail(path, e.what());
    }
    return m;
}

PinBox parse_pin(const json& v, const std::string& path) {
    ObjectReader r(v, path);
    PinBox pin;
    pin.min = to_vec3(r.require("min"), r.at("min"));
    pin.max = to_vec3(r.require("max"), r.at("max"));
    r.finish();
    if ((pin.min.array() > pin.max.array()).any()) fail(path, "min must not exceed max");
    return pin;
}

ScenarioBody parse_body(const json& v, const std::string& path) {
    ObjectReader r(v, path);
    ScenarioBody b;
    b.mesh = parse_mesh(r.require("mesh"), r.at("mesh"));
    if (const json* x = r.find("material")) b.material = parse_material(*x, r.at("material"));
    if (const json* x = r.find("position")) b.position = to_vec3(*x, r.at("position"));
    if (const json* x = r.find("rotation")) b.rotation = to_rotation(*x, r.at("rotation"));
    if (const json* x = r.find("velocity")) b.velocity = to_vec3(*x, r.at("velocity"));
    if (const json* x = r.find("angular_velocity")) {
        b.angular_velocity = to_vec3(*x, r.at("angular_velocity"));
    }
    if (const json* x = r.find("pins")) {
        const std::string ppath = r.at("pins");
        if (!x->is_array()) fail(ppath, "must be an array");
        for (size_t i = 0; i < x->size(); ++i) {
            b.pins.push_back(parse_pin((*x)[i], ppath + "[" + std::to_string(i) + "]"));
        }
    }
    r.finish();
    return b;
}

Collider parse_collider(const json& v, const std::string& path) {
    ObjectReader r(v, path);
    const std::string type = to_string(r.require("type"), r.at("type"));
    Real friction = 0.0f;
    if (const json* x = r.find("friction")) {
        friction = to_real(*x, r.at("friction"));
        if (friction < 0.0f) fail(r.at("friction"), "must not be negative");
    }
    Collider c;
    if (type == "plane") {
        const Vec3 point = to_vec3(r.require("point"), r.at("point"));
        const Vec3 normal = to_vec3(r.require("normal"), r.at("normal"));
        if (!(normal.cast<double>().norm() > 0.0)) fail(r.at("normal"), "must be non-zero");
        r.finish();
        c = Collider::plane(point, normal, friction);
    } else if (type == "sphere") {
        const Vec3 center = to_vec3(r.require("center"), r.at("center"));
        const Real radius = to_positive_real(r.require("radius"), r.at("radius"));
        r.finish();
        c = Collider::sphere(center, radius, friction);
    } else if (type == "box") {
        const Vec3 center = to_vec3(r.require("center"), r.at("center"));
        const Vec3 half = to_positive_vec3(r.require("half_extents"), r.at("half_extents"));
        Mat3 rotation = Mat3::Identity();
        if (const json* x = r.find("rotation")) rotation = to_rotation(*x, r.at("rotation"));
        r.finish();
        c = Collider::box(center, half, rotation, friction);
    } else {
        fail(r.at("type"), "must be \"plane\", \"sphere\" or \"box\"");
    }
    return c;
}

// Preset counts, then optional overrides of each setting.
void parse_solver(const json* v, const std::string& path, Scenario& s) {
    s.preset = "realtime";
    s.solver = XpbdSettings::realtime();
    if (v == nullptr) return;
    ObjectReader r(*v, path);
    if (const json* x = r.find("preset")) {
        s.preset = to_string(*x, r.at("preset"));
        if (s.preset == "reference") {
            s.solver = XpbdSettings::reference();
        } else if (s.preset != "realtime") {
            fail(r.at("preset"), "must be \"realtime\" or \"reference\"");
        }
    }
    if (const json* x = r.find("substeps")) s.solver.substeps = to_count(*x, r.at("substeps"));
    if (const json* x = r.find("iterations")) {
        s.solver.iterations = to_count(*x, r.at("iterations"));
    }
    if (const json* x = r.find("gravity")) s.solver.gravity = to_vec3(*x, r.at("gravity"));
    r.finish();
}

}  // namespace

Scenario parse_scenario(const std::string& json_text) {
    json doc;
    try {
        doc = json::parse(json_text);
    } catch (const json::parse_error& e) {
        throw std::invalid_argument(std::string("scenario: invalid JSON: ") + e.what());
    }

    ObjectReader r(doc, "");
    Scenario s;
    const Index version = to_index(r.require("format_version"), "format_version");
    if (version != kScenarioFormatVersion) {
        fail("format_version", "unsupported version " + std::to_string(version) +
                                   " (expected " + std::to_string(kScenarioFormatVersion) + ")");
    }
    s.name = to_string(r.require("name"), "name");
    if (s.name.empty()) fail("name", "must not be empty");
    const json& seed = r.require("seed");
    if (!seed.is_number_unsigned()) fail("seed", "must be a non-negative integer");
    s.seed = seed.get<std::uint64_t>();
    s.frames = to_count(r.require("frames"), "frames");
    parse_solver(r.find("solver"), "solver", s);

    const json& bodies = r.require("bodies");
    if (!bodies.is_array()) fail("bodies", "must be an array");
    if (bodies.empty()) fail("bodies", "must contain at least one body");
    for (size_t i = 0; i < bodies.size(); ++i) {
        s.bodies.push_back(parse_body(bodies[i], "bodies[" + std::to_string(i) + "]"));
    }

    if (const json* cols = r.find("colliders")) {
        if (!cols->is_array()) fail("colliders", "must be an array");
        for (size_t i = 0; i < cols->size(); ++i) {
            s.colliders.push_back(
                parse_collider((*cols)[i], "colliders[" + std::to_string(i) + "]"));
        }
    }
    r.finish();
    return s;
}

Scenario load_scenario(const std::string& path) {
    std::ifstream in(path, std::ios::binary);
    if (!in) throw std::runtime_error("cannot read scenario file: " + path);
    std::ostringstream text;
    text << in.rdbuf();
    if (in.bad()) throw std::runtime_error("cannot read scenario file: " + path);
    return parse_scenario(text.str());
}

}  // namespace hp
