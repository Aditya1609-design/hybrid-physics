#include <doctest/doctest.h>

#include <cmath>
#include <limits>
#include <stdexcept>

#include "hp/collision/collider.hpp"
#include "hp/core/rng.hpp"

namespace {

using V3d = Eigen::Vector3d;

double sdf(const hp::Collider& c, const V3d& p, V3d* normal = nullptr) {
    V3d n;
    const double d = hp::signed_distance(c, p, n);
    if (normal) *normal = n;
    return d;
}

// Central-difference gradient of the signed distance.
V3d numeric_gradient(const hp::Collider& c, const V3d& p) {
    const double eps = 1e-6;
    V3d g;
    for (int k = 0; k < 3; ++k) {
        V3d a = p, b = p;
        a[k] += eps;
        b[k] -= eps;
        g[k] = (sdf(c, a) - sdf(c, b)) / (2.0 * eps);
    }
    return g;
}

// Rotation about a unit axis by `angle` radians.
hp::Mat3 rotation(const hp::Vec3& axis, float angle) {
    return Eigen::AngleAxis<float>(angle, axis.normalized()).toRotationMatrix();
}

}  // namespace

TEST_CASE("plane signed distance and normal") {
    const hp::Collider ground = hp::Collider::plane(hp::Vec3(0, -1, 0), hp::Vec3(0, 2, 0), 0.5f);
    CHECK(ground.normal.isApprox(hp::Vec3::UnitY()));
    V3d n;
    CHECK(sdf(ground, V3d(3, 0.5, -2), &n) == doctest::Approx(1.5));
    CHECK(n.isApprox(V3d::UnitY()));
    CHECK(sdf(ground, V3d(0, -1.25, 0)) == doctest::Approx(-0.25));

    // Tilted plane through the origin.
    const hp::Collider slope = hp::Collider::plane(hp::Vec3::Zero(), hp::Vec3(1, 1, 0), 0.0f);
    CHECK(sdf(slope, V3d(1, 1, 0)) == doctest::Approx(std::sqrt(2.0)).epsilon(1e-6));
}

TEST_CASE("sphere signed distance and normal") {
    const hp::Collider s = hp::Collider::sphere(hp::Vec3(1, 0, 0), 0.5f, 0.0f);
    V3d n;
    CHECK(sdf(s, V3d(1, 2, 0), &n) == doctest::Approx(1.5));
    CHECK(n.isApprox(V3d::UnitY()));
    CHECK(sdf(s, V3d(1.25, 0, 0), &n) == doctest::Approx(-0.25));
    CHECK(n.isApprox(V3d::UnitX()));
    // The exact centre still gives a unit normal.
    CHECK(sdf(s, V3d(1, 0, 0), &n) == doctest::Approx(-0.5));
    CHECK(n.norm() == doctest::Approx(1.0));
}

TEST_CASE("axis-aligned box: faces, edges, corners and inside") {
    const hp::Collider b =
        hp::Collider::box(hp::Vec3::Zero(), hp::Vec3(1, 2, 3), hp::Mat3::Identity(), 0.0f);
    V3d n;
    // Outside a face.
    CHECK(sdf(b, V3d(1.5, 0, 0), &n) == doctest::Approx(0.5));
    CHECK(n.isApprox(V3d::UnitX()));
    // Outside an edge and a corner: Euclidean distance to the closest point.
    CHECK(sdf(b, V3d(2, 3, 0), &n) == doctest::Approx(std::sqrt(2.0)));
    CHECK(n.isApprox(V3d(1, 1, 0).normalized()));
    CHECK(sdf(b, V3d(-2, -3, -4), &n) == doctest::Approx(std::sqrt(3.0)));
    CHECK(n.isApprox(V3d(-1, -1, -1).normalized()));
    // Inside: distance to the nearest face, which here is -z (0.5 away).
    CHECK(sdf(b, V3d(0, 0, -2.5), &n) == doctest::Approx(-0.5));
    CHECK(n.isApprox(-V3d::UnitZ()));
    // On the surface.
    CHECK(sdf(b, V3d(0.3, 2.0, 0.1)) == doctest::Approx(0.0));
}

TEST_CASE("rotated box matches the same box in its own frame") {
    const hp::Vec3 c(0.5f, 1.0f, -0.25f);
    const hp::Mat3 r = rotation(hp::Vec3(1, 2, 3), 0.7f);
    const hp::Collider b = hp::Collider::box(c, hp::Vec3(0.4f, 0.3f, 0.2f), r, 0.0f);
    const hp::Collider local =
        hp::Collider::box(hp::Vec3::Zero(), hp::Vec3(0.4f, 0.3f, 0.2f), hp::Mat3::Identity(), 0.0f);
    hp::Rng rng(7);
    for (int i = 0; i < 200; ++i) {
        const V3d q(rng.uniform(-1.0f, 1.0f), rng.uniform(-1.0f, 1.0f), rng.uniform(-1.0f, 1.0f));
        const V3d p = c.cast<double>() + r.cast<double>() * q;
        V3d n, nl;
        CHECK(sdf(b, p, &n) == doctest::Approx(sdf(local, q, &nl)).epsilon(1e-5));
        CHECK((n - r.cast<double>() * nl).norm() < 1e-5);
    }
}

TEST_CASE("normals are unit length and match the distance gradient") {
    const hp::Collider colliders[] = {
        hp::Collider::plane(hp::Vec3(0, 0.1f, 0), hp::Vec3(0.2f, 1, -0.1f), 0.0f),
        hp::Collider::sphere(hp::Vec3(0.1f, -0.2f, 0.3f), 0.6f, 0.0f),
        hp::Collider::box(hp::Vec3(0.1f, 0, 0), hp::Vec3(0.5f, 0.3f, 0.4f),
                          rotation(hp::Vec3(0, 1, 1), 0.4f), 0.0f),
    };
    hp::Rng rng(11);
    for (const hp::Collider& c : colliders) {
        for (int i = 0; i < 300; ++i) {
            const V3d p(rng.uniform(-1.5f, 1.5f), rng.uniform(-1.5f, 1.5f),
                        rng.uniform(-1.5f, 1.5f));
            V3d n;
            sdf(c, p, &n);
            // Normals and rotations are stored in float32, so unit length holds to ~1e-7.
            CHECK(n.norm() == doctest::Approx(1.0).epsilon(1e-6));
            // The SDF has kinks inside a box (where the nearest face changes); random points
            // land on them with probability 0, so the gradient check holds everywhere sampled.
            CHECK((numeric_gradient(c, p) - n).norm() < 1e-4);
        }
    }
}

TEST_CASE("float and double evaluation agree") {
    const hp::Collider b = hp::Collider::box(hp::Vec3(0, 1, 0), hp::Vec3(1, 1, 1),
                                             rotation(hp::Vec3(1, 0, 0), 0.3f), 0.2f);
    hp::Vec3 nf;
    V3d nd;
    const hp::Vec3 p(0.3f, 2.4f, -0.2f);
    const float df = hp::signed_distance(b, p, nf);
    const double dd = hp::signed_distance(b, V3d(p.cast<double>()), nd);
    CHECK(df == doctest::Approx(dd).epsilon(1e-6));
    CHECK((nf.cast<double>() - nd).norm() < 1e-6);
}

TEST_CASE("invalid colliders are rejected") {
    const float nan = std::numeric_limits<float>::quiet_NaN();
    CHECK_THROWS_AS(hp::Collider::plane(hp::Vec3::Zero(), hp::Vec3::Zero(), 0.0f),
                    std::invalid_argument);
    CHECK_THROWS_AS(hp::Collider::plane(hp::Vec3::Zero(), hp::Vec3::UnitY(), -0.1f),
                    std::invalid_argument);
    CHECK_THROWS_AS(hp::Collider::plane(hp::Vec3(nan, 0, 0), hp::Vec3::UnitY(), 0.0f),
                    std::invalid_argument);
    CHECK_THROWS_AS(hp::Collider::sphere(hp::Vec3::Zero(), 0.0f, 0.0f), std::invalid_argument);
    CHECK_THROWS_AS(hp::Collider::sphere(hp::Vec3::Zero(), 1.0f, nan), std::invalid_argument);
    CHECK_THROWS_AS(
        hp::Collider::box(hp::Vec3::Zero(), hp::Vec3(1, 0, 1), hp::Mat3::Identity(), 0.0f),
        std::invalid_argument);
    hp::Mat3 mirror = hp::Mat3::Identity();
    mirror(0, 0) = -1.0f;
    CHECK_THROWS_AS(hp::Collider::box(hp::Vec3::Zero(), hp::Vec3::Ones(), mirror, 0.0f),
                    std::invalid_argument);
    CHECK_THROWS_AS(
        hp::Collider::box(hp::Vec3::Zero(), hp::Vec3::Ones(), 2.0f * hp::Mat3::Identity(), 0.0f),
        std::invalid_argument);

    // Hand-edited fields are caught by validate().
    hp::Collider c = hp::Collider::sphere(hp::Vec3::Zero(), 1.0f, 0.0f);
    c.radius = -1.0f;
    CHECK_THROWS_AS(c.validate(), std::invalid_argument);
}
