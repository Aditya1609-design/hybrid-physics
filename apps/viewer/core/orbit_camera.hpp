#pragma once

#include "hp/core/types.hpp"

namespace hp::viewer {

// Camera that circles a target point at a given distance. Yaw turns about +y, pitch tilts
// up from the horizontal plane (positive = camera above the target).
struct OrbitCamera {
    static constexpr float kMaxPitch = 1.45f;  // about 83 degrees, short of the poles
    static constexpr float kMinDistance = 0.05f;
    static constexpr float kMaxDistance = 500.0f;

    Vec3 target = Vec3::Zero();
    float yaw = 0.6f;
    float pitch = 0.45f;
    float distance = 4.0f;
    float fovy_deg = 45.0f;  // vertical field of view

    Vec3 position() const;
    Vec3 forward() const;  // unit vector from the camera towards the target
    Vec3 right() const;    // unit screen-right direction
    Vec3 up() const;       // unit screen-up direction

    // Turns by the given angles in radians; pitch is clamped to +-kMaxPitch.
    void orbit(float d_yaw, float d_pitch);
    // Multiplies the distance by `factor`, clamped to [kMinDistance, kMaxDistance].
    void zoom(float factor);
    // Moves the target so the scene follows a mouse drag of (dx, dy) pixels (y down)
    // in a viewport `viewport_height` pixels tall.
    void pan(float dx, float dy, float viewport_height);
    // Centres on the box [lo, hi] and backs off until it fits the view, keeping the angles.
    void frame(const Vec3& lo, const Vec3& hi, float aspect);
};

}  // namespace hp::viewer
