#include <doctest/doctest.h>

#include <cmath>

#include "core/orbit_camera.hpp"

using hp::Vec3;
using hp::viewer::OrbitCamera;

namespace {

constexpr float kPi = 3.14159265358979f;

void check_orthonormal(const OrbitCamera& cam) {
    const Vec3 f = cam.forward();
    const Vec3 r = cam.right();
    const Vec3 u = cam.up();
    CHECK(f.norm() == doctest::Approx(1.0f).epsilon(1e-5));
    CHECK(r.norm() == doctest::Approx(1.0f).epsilon(1e-5));
    CHECK(u.norm() == doctest::Approx(1.0f).epsilon(1e-5));
    CHECK(std::abs(f.dot(r)) < 1e-5f);
    CHECK(std::abs(f.dot(u)) < 1e-5f);
    CHECK(std::abs(r.dot(u)) < 1e-5f);
    CHECK(u.y() > 0.0f);  // screen-up never points below the horizon
}

}  // namespace

TEST_CASE("orbit camera keeps an orthonormal basis and clamps pitch short of the poles") {
    OrbitCamera cam;
    cam.target = Vec3(1, 2, 3);
    for (float yaw : {-3.0f, -1.0f, 0.0f, 0.5f, 2.9f}) {
        for (float pitch : {-1.4f, -0.3f, 0.0f, 0.7f, 1.4f}) {
            cam.yaw = yaw;
            cam.pitch = pitch;
            CAPTURE(yaw);
            CAPTURE(pitch);
            check_orthonormal(cam);
            const Vec3 back = cam.position() + cam.distance * cam.forward();
            CHECK((back - cam.target).norm() < 1e-5f);
        }
    }

    cam.pitch = 0.0f;
    cam.orbit(0.0f, 10.0f);
    CHECK(cam.pitch == OrbitCamera::kMaxPitch);
    check_orthonormal(cam);
    cam.orbit(0.0f, -20.0f);
    CHECK(cam.pitch == -OrbitCamera::kMaxPitch);
    check_orthonormal(cam);

    // Yaw wraps instead of growing without bound.
    cam.orbit(100.0f * kPi + 0.25f, 0.0f);
    CHECK(std::abs(cam.yaw) <= kPi);
}

TEST_CASE("orbit camera zoom stays within its distance limits") {
    OrbitCamera cam;
    cam.distance = 2.0f;
    cam.zoom(0.5f);
    CHECK(cam.distance == doctest::Approx(1.0f));
    cam.zoom(1e-9f);
    CHECK(cam.distance == OrbitCamera::kMinDistance);
    cam.zoom(1e9f);
    CHECK(cam.distance == OrbitCamera::kMaxDistance);
}

TEST_CASE("framing a box puts every corner inside the view") {
    const Vec3 lo(-0.3f, 0.1f, -1.2f);
    const Vec3 hi(1.7f, 0.6f, 0.4f);
    for (float aspect : {0.5f, 1.0f, 1.6f, 2.5f}) {
        for (float yaw : {0.0f, 0.8f, 2.5f}) {
            OrbitCamera cam;
            cam.yaw = yaw;
            cam.frame(lo, hi, aspect);
            CAPTURE(aspect);
            CAPTURE(yaw);
            CHECK((cam.target - 0.5f * (lo + hi)).norm() < 1e-6f);
            const float tan_v = std::tan(0.5f * cam.fovy_deg * kPi / 180.0f);
            const float tan_h = tan_v * aspect;
            for (int i = 0; i < 8; ++i) {
                const Vec3 corner((i & 1) ? hi.x() : lo.x(), (i & 2) ? hi.y() : lo.y(),
                                  (i & 4) ? hi.z() : lo.z());
                const Vec3 d = corner - cam.position();
                const float z = d.dot(cam.forward());
                REQUIRE(z > 0.0f);
                CHECK(std::abs(d.dot(cam.right())) / z <= tan_h);
                CHECK(std::abs(d.dot(cam.up())) / z <= tan_v);
            }
        }
    }
}

TEST_CASE("panning moves the target along the screen axes with the cursor") {
    OrbitCamera cam;
    cam.distance = 3.0f;
    const float viewport = 800.0f;
    const float metres_per_px =
        2.0f * cam.distance * std::tan(0.5f * cam.fovy_deg * kPi / 180.0f) / viewport;

    // Dragging right moves the scene right, so the target moves left.
    Vec3 before = cam.target;
    cam.pan(100.0f, 0.0f, viewport);
    Vec3 moved = cam.target - before;
    CHECK(moved.dot(cam.right()) == doctest::Approx(-100.0f * metres_per_px).epsilon(1e-4));
    CHECK(std::abs(moved.dot(cam.up())) < 1e-6f);

    // Dragging down (screen y grows downwards) moves the target up.
    before = cam.target;
    cam.pan(0.0f, 50.0f, viewport);
    moved = cam.target - before;
    CHECK(moved.dot(cam.up()) == doctest::Approx(50.0f * metres_per_px).epsilon(1e-4));
    CHECK(std::abs(moved.dot(cam.right())) < 1e-6f);

    // A zero-height viewport is ignored rather than dividing by zero.
    before = cam.target;
    cam.pan(10.0f, 10.0f, 0.0f);
    CHECK(cam.target == before);
}
