#include "hp/solver/detail/xpbd_kernel.hpp"

#include <algorithm>

namespace hp::detail {

template <typename S>
void XpbdKernel<S>::step(const XpbdConstraints& c, std::vector<V3>& x, std::vector<V3>& v,
                         const std::vector<S>& w, S dt, Index substeps, Index iterations,
                         const V3& gravity, S damping,
                         const std::vector<Collider>& colliders) {
    const size_t n = x.size();
    dx_.resize(n);
    lambda_edge_.resize(c.edges.size());
    lambda_tet_.resize(c.tets.size());

    const S h = dt / static_cast<S>(substeps);
    const S h2 = h * h;
    // Implicit form of linear damping: stable for any h, unlike 1 - damping * h.
    const S damp = S(1) / (S(1) + damping * h);

    for (Index sub = 0; sub < substeps; ++sub) {
        // Predict with explicit gravity; pinned vertices stay put.
        for (size_t i = 0; i < n; ++i) {
            dx_[i] = w[i] > S(0) ? V3(h * v[i] + h2 * gravity) : V3::Zero();
        }
        std::fill(lambda_edge_.begin(), lambda_edge_.end(), S(0));
        std::fill(lambda_tet_.begin(), lambda_tet_.end(), S(0));

        for (Index it = 0; it < iterations; ++it) {
            project_edges(c, x, w, h2);
            project_tets(c, x, w, h2);
            // Contacts last, so each substep ends with vertices outside the colliders.
            if (!colliders.empty()) project_contacts(colliders, x, w);
        }

        for (size_t i = 0; i < n; ++i) {
            x[i] += dx_[i];
            v[i] = damp * (dx_[i] / h);
        }
    }
}

// Distance constraints C = |p_i - p_j| - L with the XPBD multiplier update.
template <typename S>
void XpbdKernel<S>::project_edges(const XpbdConstraints& c, const std::vector<V3>& x,
                                  const std::vector<S>& w, S h2) {
    for (size_t e = 0; e < c.edges.size(); ++e) {
        const auto i = static_cast<size_t>(c.edges[e][0]);
        const auto j = static_cast<size_t>(c.edges[e][1]);
        const S wsum = w[i] + w[j];
        const S alpha = static_cast<S>(c.edge_compliance[e]) / h2;
        if (wsum + alpha == S(0)) continue;

        const V3 d = (x[i] - x[j]) + (dx_[i] - dx_[j]);
        const S len = d.norm();
        if (len == S(0)) continue;  // direction undefined; neighbours will separate it
        const S C = len - static_cast<S>(c.edge_rest_length[e]);
        const S dlambda = (-C - alpha * lambda_edge_[e]) / (wsum + alpha);
        const V3 corr = (dlambda / len) * d;
        dx_[i] += w[i] * corr;
        dx_[j] -= w[j] * corr;
        lambda_edge_[e] += dlambda;
    }
}

// Volume constraints C = V - V0, with V the signed tet volume.
template <typename S>
void XpbdKernel<S>::project_tets(const XpbdConstraints& c, const std::vector<V3>& x,
                                 const std::vector<S>& w, S h2) {
    const S sixth = S(1) / S(6);
    for (size_t t = 0; t < c.tets.size(); ++t) {
        const Tet& tet = c.tets[t];
        const auto i0 = static_cast<size_t>(tet[0]);
        const auto i1 = static_cast<size_t>(tet[1]);
        const auto i2 = static_cast<size_t>(tet[2]);
        const auto i3 = static_cast<size_t>(tet[3]);

        // Edge vectors from vertex 0, each formed as rest offset plus displacement offset.
        const V3 p0 = dx_[i0];
        const V3 e1 = (x[i1] - x[i0]) + (dx_[i1] - p0);
        const V3 e2 = (x[i2] - x[i0]) + (dx_[i2] - p0);
        const V3 e3 = (x[i3] - x[i0]) + (dx_[i3] - p0);

        const V3 g1 = sixth * e2.cross(e3);
        const V3 g2 = sixth * e3.cross(e1);
        const V3 g3 = sixth * e1.cross(e2);
        const V3 g0 = -(g1 + g2 + g3);

        const S wgrad = w[i0] * g0.squaredNorm() + w[i1] * g1.squaredNorm() +
                        w[i2] * g2.squaredNorm() + w[i3] * g3.squaredNorm();
        const S alpha = static_cast<S>(c.tet_compliance[t]) / h2;
        if (wgrad + alpha == S(0)) continue;

        const S C = e1.dot(g1) - static_cast<S>(c.tet_rest_volume[t]);  // e1 . g1 = V
        const S dlambda = (-C - alpha * lambda_tet_[t]) / (wgrad + alpha);
        dx_[i0] += (w[i0] * dlambda) * g0;
        dx_[i1] += (w[i1] * dlambda) * g1;
        dx_[i2] += (w[i2] * dlambda) * g2;
        dx_[i3] += (w[i3] * dlambda) * g3;
        lambda_tet_[t] += dlambda;
    }
}

// Hard, one-sided contact C = sdf(p) >= 0 against static colliders (infinite mass), so the
// XPBD update reduces to moving a penetrating vertex out along the collider normal.
// Positional Coulomb friction then limits the vertex's tangential motion over the substep:
// it is removed if it is within mu times the push-out (stick), otherwise shortened by that much.
template <typename S>
void XpbdKernel<S>::project_contacts(const std::vector<Collider>& colliders,
                                     const std::vector<V3>& x, const std::vector<S>& w) {
    for (size_t i = 0; i < x.size(); ++i) {
        if (w[i] == S(0)) continue;
        for (const Collider& col : colliders) {
            V3 n;
            const S d = signed_distance(col, V3(x[i] + dx_[i]), n);
            if (d >= S(0)) continue;
            dx_[i] -= d * n;

            const S limit = static_cast<S>(col.friction) * -d;
            if (limit == S(0)) continue;
            const V3 tangent = dx_[i] - dx_[i].dot(n) * n;
            const S slip = tangent.norm();
            dx_[i] -= slip <= limit ? tangent : V3((limit / slip) * tangent);
        }
    }
}

template class XpbdKernel<float>;
template class XpbdKernel<double>;

}  // namespace hp::detail
