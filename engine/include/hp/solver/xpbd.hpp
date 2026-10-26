#pragma once

#include "hp/core/sim_state.hpp"
#include "hp/core/types.hpp"
#include "hp/solver/detail/xpbd_kernel.hpp"
#include "hp/solver/solver.hpp"
#include "hp/solver/xpbd_constraints.hpp"

namespace hp {

// Frame timestep used by both presets (fixed; the solver never reads a clock).
constexpr Real kFrameDt = 1.0f / 60.0f;

// Solver settings. The two presets run the same code and differ only in these counts.
struct XpbdSettings {
    Index substeps = 1;
    Index iterations = 1;  // Gauss-Seidel sweeps per substep
    Vec3 gravity = Vec3(0.0f, -9.81f, 0.0f);

    // Slow, close to converged: generates training data and is the error reference.
    static XpbdSettings reference();
    // Fast, slightly too soft: the normal engine and the pure-classical baseline.
    static XpbdSettings realtime();

    // Throws std::invalid_argument for counts below 1 or non-finite gravity.
    void validate() const;
};

// Substepped XPBD ("small steps") with distance and volume constraints. Single-threaded and
// deterministic: constraints are always projected in the same order.
class XpbdSolver final : public ISolver {
public:
    // Throws std::invalid_argument if the material or settings are invalid.
    XpbdSolver(XpbdConstraints constraints, const Material& material,
               const XpbdSettings& settings);

    // Advances one frame of `dt`. The mask must cover all vertices (mixed ownership comes
    // later); throws std::invalid_argument otherwise, or if dt is not positive, or if the state
    // does not match the constraints.
    void step(SimState& state, const RegionMask& mask, Real dt) override;

    const XpbdConstraints& constraints() const { return constraints_; }
    const XpbdSettings& settings() const { return settings_; }

private:
    XpbdConstraints constraints_;
    XpbdSettings settings_;
    Real damping_;
    Index min_vertices_ = 0;  // one more than the largest vertex index used by a constraint
    detail::XpbdKernel<Real> kernel_;
};

}  // namespace hp
