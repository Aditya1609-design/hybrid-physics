# Phase 3: XPBD soft body

Status: approved 2026-10-27 (tag `phase-03-approved`). Branch `phase/03-xpbd`.

## What was built

- `solver`: `ISolver::step(SimState&, const RegionMask&, dt)`; `RegionMask` (full masks only);
  `Material` (density, edge and volume compliance per unit volume, damping); `build_constraints`
  (one distance constraint per edge, one volume constraint per tet, compliance scaled per
  element); `XpbdSolver` (substepped XPBD, Gauss–Seidel, explicit gravity, implicit linear
  damping, cancellation-free velocity); `XpbdSettings::realtime()` (10 × 1) and
  `::reference()` (256 × 1); fixed frame step `kFrameDt = 1/60 s`.
- `detail::XpbdKernel<S>`: the step as a template, built for float (engine) and double (precision
  tests only).
- `metrics`: `constraint_error` (RMS and max relative edge and volume error, inverted tets) and
  `energy` (kinetic, gravitational, elastic), both computed in double.
- `scene`: `pin_vertices(state, predicate)`.
- `apps/benchmark/hp_xpbd_speed`: per-frame timing of both presets, 729 to 50,653 vertices.
- `ml/tools/gnn_cost_estimate.py` and `ml/requirements.txt` (torch 2.14.1, numpy 2.5.4).
- ADR 0005 (solver, presets, precision, limits); speed results in `results/phase-03-speed/`.

## How to run

```sh
./scripts/ci.sh                                         # release + asan, 58 tests each
./build/release/tests/hp_tests -tc="*preset*,*converge*,*float32*" -s   # prints the measured numbers
./build/release/apps/benchmark/hp_xpbd_speed --quick    # about 10 s; without --quick 1-2 min
python3 -m venv .venv && .venv/bin/python -m pip install -r ml/requirements.txt
.venv/bin/python ml/tools/gnn_cost_estimate.py --flops-only   # instant, no PyTorch needed
.venv/bin/python ml/tools/gnn_cost_estimate.py --cells 8 10 12 \
    --xpbd-csv results/phase-03-speed/xpbd_speed.csv    # CPU + MPS timing, about a minute
```

## Verification done

| Check | Result |
|-------|--------|
| `ci.sh` before every commit (9 commits) | PASS, release and asan, no warnings |
| Uniform stretch: elastic energy at 3, 6, 12 cells per side | 511.14 J each (resolution-independent) |
| Free fall vs the discrete integrator | matches to 1e-4 relative; shape error < 1e-5 |
| Linear momentum, zero gravity, deforming and spinning body | conserved to 1e-4 relative |
| Rerun, and a second solver instance | byte-identical positions and velocities |
| Convergence, damped stiff cantilever, 2 → 256 substeps | edge error 3.0e-2 → 1.1e-3, monotonic |
| Reference vs real-time edge error, rigid beam | 1.9e-6 vs 1.2e-3 (650×) |
| Reference vs real-time edge error, `Y = 100 MPa` | 1.1e-3 vs 2.5e-3 (2.3×; the true answer also stretches) |
| Reference vs 2× its substeps, undamped swinging beam | 3.3e-5 of beam length |
| float32 vs double kernel, 30 frames | 4.6e-7 (real-time) and 9.1e-6 (reference) of beam length |
| Volume constraints on/off under load | volume error 4.1e-4 vs 0.21 |
| 50% squash (soft material) | volume error 0.5 → 1.8e-5 |
| Locally inverted tets | none left after 2 frames; back to rest shape |
| Real-time, undamped, 1000 frames | finite; energy never rises above start + 8e-5 of the energy exchanged |

## Speed check (details and caveats in `results/phase-03-speed/README.md`)

- XPBD costs about 25 ns per constraint projection, linear in mesh size. Real-time: 1.55 ms at 729
  vertices, 12 ms at 4.9k, 137 ms at 50k. Reference: 26× the real-time cost.
- Untrained MeshGraphNets (128 latent, 15 steps), best of CPU/MPS: 52 ms at 729 vertices,
  5.4 s at 50k. That is **31–40× slower than real-time XPBD at every size, with no crossover**,
  and 1.2–1.5× slower than the reference preset. Smaller variants: 64x10 is 10–14× slower than
  real-time but about 2× faster than reference; 64x5 is 5–7× slower than real-time and about 4×
  faster than reference.
- Honest reading: a per-step speed win over the real-time solver is not plausible on this
  hardware. Possible claims are (a) near-reference accuracy for less than the reference cost,
  with a mid-sized model, or (b) one GNN step covering several frames. Both are untested until
  GNN training and the speed-crossover check.

## Changes from the approved plan

- Preset values from measurement: real-time 10 × 1 (plan: about 8 × 1), reference 256 × 1 (plan's
  expected range 64–256). One iteration for both, because extra substeps help more than extra
  iterations in most cases.
- Preset test thresholds: the plan said "reference at most ⅕ of real-time, to be confirmed". The
  measured ratio is 650× for a rigid beam but only 2.3× for a compliant one (real stretch sets a
  floor), so the tests require 100× and 2× respectively.
- Volume constraint is `C = V − V0` rather than `6(V − V0)`; the compliance scaling absorbs the
  factor.
- `Material` and `XpbdConstraints` live in `hp/solver/xpbd_constraints.hpp`. The kernel template
  is a public `detail` header rather than internal, so the precision test can use the double
  build without reaching into `src/`.
- The planned "squashed to 50%" volume test with the default material was not usable: it stores
  1.3 MJ and explodes. The test uses a softer material. A mirrored-body test was replaced by a
  local-inversion test (see limitations).
- The energy test tolerance is relative to the energy exchanged, because the total starts near 0 J.

## Known limitations

- **Real-time accuracy depends strongly on the material.** Tip deflection against reference:
  +1.8% at `Y = 1 MPa`, +58% at 10 MPa, 6.4× at 100 MPa (ADR 0005 table).
- A fully mirrored body un-inverts but gets stuck in a tangled local minimum; distance-plus-volume
  constraints are not inversion-robust. Local inversions recover.
- The "Young's" and "bulk" moduli belong to an edge network, not a continuum model; the
  Poisson ratio is not directly set.
- Angular momentum is only approximately conserved (discretization error, shrinks with substeps).
- The edge-strain metric mixes real stretch with solver error for compliant materials; position
  error against the reference is the better measure there.
- `RegionMask` supports only full masks.
- XPBD is not optimized (scalar, single-threaded).
- The GNN timing is eager PyTorch with an untrained model; see the caveats in the results file.

## What to evaluate

- Run `./scripts/ci.sh` on the teammate's Mac (`CI PASS`), and `hp_xpbd_speed --quick` to compare
  timings.
- Whether the preset values (10 × 1, 256 × 1) and the reference criterion (doubling moves results
  by < 0.1% of body size) suit you. The reference cost sets the dataset generation time: about
  12 s per 300-frame trajectory at 750 vertices, single-threaded.
- The material-dependence table in ADR 0005: which stiffness range the dataset should cover.
- The speed verdict in `results/phase-03-speed/README.md`, and which claim to aim for.
- The per-element compliance scaling and its resolution-independence test.

## Decisions needed

1. **Project claim, given the speed check.** The original premise (GNN regions faster than
   XPBD regions) does not hold against the real-time preset at any size we measured.
   Recommendation: keep the phase order, but frame the target as "near-reference accuracy at
   lower cost than the reference preset". Under that framing, the hybrid uses the GNN where
   real-time XPBD is inaccurate (stiff materials). Also keep "one GNN step per several frames"
   as a second option to test in GNN training. The final decision belongs to the speed-crossover
   phase. Changing the framing also means updating the project plan documents.
2. **Material range for later scenarios and the dataset.** The real-time preset is accurate only
   for soft materials. Recommendation: cover `Y` from 0.3 to 30 MPa, so the baseline is
   neither trivially right nor hopeless.

## Review outcome

- Approved, including the preset values (real-time 10 × 1, reference 256 × 1) and the changes
  from the plan listed above.
- Working direction: near-reference accuracy for less than the reference preset's cost. Small
  GNNs (for example 64x10 and 64x5) and GNN steps that cover several frames will both be tested.
  The project plan documents stay unchanged for now; the speed-crossover phase makes the final call.
- Material range: Young's-like modulus `Y` from 0.3 to 30 MPa for scenarios and the
  training/validation/test sets. The out-of-distribution test set also gets a few stiffer
  values, for example 100 MPa.
- Note for GNN training: start with a quick accuracy test of a small GNN on the simplest scene,
  before any full training run, to check early that a model of that size can get close to the
  reference preset.
