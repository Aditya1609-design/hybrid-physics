#pragma once

#include <cstdint>

#include "hp/core/types.hpp"

namespace hp {

// 3x3 float matrix with no alignment requirement, so colliders can sit in a std::vector.
using Mat3 = Eigen::Matrix<Real, 3, 3>;

enum class ColliderKind : std::uint8_t { Plane = 0, Sphere = 1, Box = 2 };

// A static collider. One plain struct for all kinds so the solver can evaluate it inline,
// in float or double, without virtual calls or allocation. Build it with the factories below.
struct Collider {
    ColliderKind kind = ColliderKind::Plane;
    Vec3 center = Vec3::Zero();         // plane: a point on it; sphere and box: the centre
    Vec3 normal = Vec3::UnitY();        // plane only: unit normal pointing out of the solid side
    Real radius = 0.0f;                 // sphere only
    Vec3 half_extents = Vec3::Zero();   // box only
    Mat3 rotation = Mat3::Identity();   // box only: columns are the box axes in world space
    Real friction = 0.0f;               // Coulomb coefficient, same for sticking and sliding

    // Half-space below the plane through `point`; `normal` is normalized here.
    static Collider plane(const Vec3& point, const Vec3& normal, Real friction);
    static Collider sphere(const Vec3& center, Real radius, Real friction);
    // Oriented box; `rotation` must be a proper rotation matrix.
    static Collider box(const Vec3& center, const Vec3& half_extents, const Mat3& rotation,
                        Real friction);

    // Throws std::invalid_argument for non-finite values, a non-unit plane normal, a
    // non-positive radius or half extent, a rotation that is not orthonormal with
    // determinant 1, or negative friction.
    void validate() const;
};

// Signed distance from p to the collider surface (negative inside) and the outward unit
// normal of the nearest surface point. Inside a box the nearest face is used; ties go to
// the lower axis. Templated so the double test kernel uses the same code.
template <typename S>
S signed_distance(const Collider& c, const Eigen::Matrix<S, 3, 1>& p,
                  Eigen::Matrix<S, 3, 1>& normal) {
    using V3 = Eigen::Matrix<S, 3, 1>;
    const V3 center = c.center.template cast<S>();
    switch (c.kind) {
        case ColliderKind::Plane: {
            normal = c.normal.template cast<S>();
            return normal.dot(p - center);
        }
        case ColliderKind::Sphere: {
            const V3 d = p - center;
            const S len = d.norm();
            // At the exact centre any direction is nearest; pick +y for determinism.
            normal = len > S(0) ? V3(d / len) : V3(V3::UnitY());
            return len - static_cast<S>(c.radius);
        }
        case ColliderKind::Box: {
            const Eigen::Matrix<S, 3, 3> r = c.rotation.template cast<S>();
            const V3 h = c.half_extents.template cast<S>();
            const V3 q = r.transpose() * (p - center);  // point in box coordinates
            const V3 gap = q.cwiseAbs() - h;            // > 0 on axes where p is outside
            if ((gap.array() > S(0)).any()) {
                // Outside: distance to the closest point on the box.
                const V3 diff = q - q.cwiseMax(-h).cwiseMin(h);
                const S len = diff.norm();
                normal = r * (diff / len);
                return len;
            }
            // Inside: leave through the nearest face.
            int axis = 0;
            for (int k = 1; k < 3; ++k) {
                if (gap[k] > gap[axis]) axis = k;
            }
            normal = (q[axis] < S(0) ? S(-1) : S(1)) * r.col(axis);
            return gap[axis];
        }
    }
    normal = V3::UnitY();
    return S(0);
}

}  // namespace hp
