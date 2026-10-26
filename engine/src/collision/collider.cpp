#include "hp/collision/collider.hpp"

#include <cmath>
#include <stdexcept>

namespace hp {

Collider Collider::plane(const Vec3& point, const Vec3& normal, Real friction) {
    Collider c;
    c.kind = ColliderKind::Plane;
    c.center = point;
    // Normalize only a usable normal; validate() rejects zero or non-finite ones.
    const Real len = normal.norm();
    c.normal = len > 0.0f && std::isfinite(len) ? Vec3(normal / len) : normal;
    c.friction = friction;
    c.validate();
    return c;
}

Collider Collider::sphere(const Vec3& center, Real radius, Real friction) {
    Collider c;
    c.kind = ColliderKind::Sphere;
    c.center = center;
    c.radius = radius;
    c.friction = friction;
    c.validate();
    return c;
}

Collider Collider::box(const Vec3& center, const Vec3& half_extents, const Mat3& rotation,
                       Real friction) {
    Collider c;
    c.kind = ColliderKind::Box;
    c.center = center;
    c.half_extents = half_extents;
    c.rotation = rotation;
    c.friction = friction;
    c.validate();
    return c;
}

void Collider::validate() const {
    if (!center.allFinite()) throw std::invalid_argument("collider centre must be finite");
    if (!(friction >= 0.0f) || !std::isfinite(friction)) {
        throw std::invalid_argument("collider friction must be >= 0 and finite");
    }
    switch (kind) {
        case ColliderKind::Plane:
            if (!normal.allFinite() || std::abs(normal.norm() - 1.0f) > 1e-4f) {
                throw std::invalid_argument("plane normal must be a finite unit vector");
            }
            return;
        case ColliderKind::Sphere:
            if (!(radius > 0.0f) || !std::isfinite(radius)) {
                throw std::invalid_argument("sphere radius must be positive and finite");
            }
            return;
        case ColliderKind::Box: {
            if (!half_extents.allFinite() || !(half_extents.minCoeff() > 0.0f)) {
                throw std::invalid_argument("box half extents must be positive and finite");
            }
            // Proper rotation: orthonormal columns, no mirroring.
            if (!rotation.allFinite() ||
                (rotation.transpose() * rotation - Mat3::Identity()).cwiseAbs().maxCoeff() >
                    1e-4f ||
                std::abs(rotation.determinant() - 1.0f) > 1e-4f) {
                throw std::invalid_argument("box rotation must be a proper rotation matrix");
            }
            return;
        }
    }
    throw std::invalid_argument("unknown collider kind");
}

}  // namespace hp
