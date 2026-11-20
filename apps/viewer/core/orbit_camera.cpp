#include "core/orbit_camera.hpp"

#include <algorithm>
#include <cmath>

namespace hp::viewer {

namespace {

constexpr float kPi = 3.14159265358979f;

float radians(float deg) { return deg * kPi / 180.0f; }

// Unit vector from the target towards the camera.
Vec3 offset_dir(float yaw, float pitch) {
    const float cp = std::cos(pitch);
    return Vec3(cp * std::sin(yaw), std::sin(pitch), cp * std::cos(yaw));
}

}  // namespace

Vec3 OrbitCamera::position() const { return target + distance * offset_dir(yaw, pitch); }

Vec3 OrbitCamera::forward() const { return -offset_dir(yaw, pitch); }

// Pitch never reaches the poles, so forward is never parallel to +y.
Vec3 OrbitCamera::right() const { return forward().cross(Vec3::UnitY()).normalized(); }

Vec3 OrbitCamera::up() const { return right().cross(forward()); }

void OrbitCamera::orbit(float d_yaw, float d_pitch) {
    yaw = std::remainder(yaw + d_yaw, 2.0f * kPi);
    pitch = std::clamp(pitch + d_pitch, -kMaxPitch, kMaxPitch);
}

void OrbitCamera::zoom(float factor) {
    distance = std::clamp(distance * factor, kMinDistance, kMaxDistance);
}

// One pixel covers this many metres at the target's depth, so the target moves with the cursor.
void OrbitCamera::pan(float dx, float dy, float viewport_height) {
    if (viewport_height <= 0.0f) return;
    const float metres_per_px =
        2.0f * distance * std::tan(0.5f * radians(fovy_deg)) / viewport_height;
    target += (-dx * metres_per_px) * right() + (dy * metres_per_px) * up();
}

// Fits the box's bounding sphere inside the narrower of the two view angles, with a margin.
void OrbitCamera::frame(const Vec3& lo, const Vec3& hi, float aspect) {
    target = 0.5f * (lo + hi);
    const float radius = std::max(0.5f * (hi - lo).norm(), 1e-3f);
    const float half_v = 0.5f * radians(fovy_deg);
    const float half_h = std::atan(std::tan(half_v) * std::max(aspect, 1e-3f));
    const float half = std::min(half_v, half_h);
    distance = std::clamp(1.15f * radius / std::sin(half), kMinDistance, kMaxDistance);
}

}  // namespace hp::viewer
