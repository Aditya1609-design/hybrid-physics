#pragma once

#include <vector>

#include "hp/collision/collider.hpp"
#include "hp/core/types.hpp"
#include "hp/solver/xpbd_constraints.hpp"

namespace hp::detail {

// The XPBD step on raw arrays, for any floating-point type. The engine uses the float
// version through XpbdSolver; the double version exists so tests can measure float32 error.
template <typename S>
class XpbdKernel {
public:
    using V3 = Eigen::Matrix<S, 3, 1>;

    // One frame of `dt` split into `substeps`, each with `iterations` Gauss-Seidel sweeps over
    // the edges, the tets and then the collider contacts. Updates x and v in place; w is the
    // inverse mass. With no colliders the result is identical to the contact-free solver.
    void step(const XpbdConstraints& c, std::vector<V3>& x, std::vector<V3>& v,
              const std::vector<S>& w, S dt, Index substeps, Index iterations, const V3& gravity,
              S damping, const std::vector<Collider>& colliders = {});

private:
    void project_edges(const XpbdConstraints& c, const std::vector<V3>& x,
                       const std::vector<S>& w, S h2);
    void project_tets(const XpbdConstraints& c, const std::vector<V3>& x,
                      const std::vector<S>& w, S h2);
    void project_contacts(const std::vector<Collider>& colliders, const std::vector<V3>& x,
                          const std::vector<S>& w);

    // Displacement of each vertex within the current substep. Positions during projection are
    // x + dx, so velocity dx / h never subtracts two nearly equal positions.
    std::vector<V3> dx_;
    std::vector<S> lambda_edge_;
    std::vector<S> lambda_tet_;
};

extern template class XpbdKernel<float>;
extern template class XpbdKernel<double>;

}  // namespace hp::detail
