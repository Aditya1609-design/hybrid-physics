# Phase 4: collider overhead

- Machine: Apple M1 (4P + 4E cores, 8 GB), macOS 27.0.1. See `machine_info.json`.
- Engine build: `hybrid-physics 0.1.0 | Release | Apple LLVM 21.0.0 (clang-2100.3.34.2) |
  fp-contract=off | sanitizers=none`, commit `79c6120`. The working tree had uncommitted docs
  only (hence `git_dirty`); the engine and benchmark code were as committed.
- File: `collider_overhead.csv` holds the raw rows. Reproduce with:
  ```sh
  ./build/release/apps/benchmark/hp_xpbd_speed --colliders --csv results/raw/collider_overhead.csv
  ```

## Setup

Same scene as the Phase 3 speed check: a unit box pinned on its x = min face, sagging under
gravity, default material, one thread, frame 1/60 s. The `+colliders` rows add a ground plane
just below the box, a sphere under its free end and an oriented box beside it, all with μ = 0.5.
During the timed frames 22–39 of the 729 vertices are in contact (measured separately).

## Frame time (median ms) and overhead of three colliders

| Vertices | Real-time | + colliders | Overhead | Reference | + colliders | Overhead |
|---------:|----------:|------------:|---------:|----------:|------------:|---------:|
| 729      | 1.57      | 1.73        | +10.5%*  | 41.3      | 42.7        | +3.4%    |
| 1,331    | 2.96      | 3.17        | +6.9%    | 76.0      | 81.6        | +7.3%    |
| 2,197    | 5.06      | 5.39        | +6.6%    | 130       | 139         | +7.1%    |
| 4,913    | 11.9      | 12.5        | +5.7%    | 304       | 324         | +6.6%    |
| 10,648   | 26.5      | 28.1        | +5.9%    | 712       | 721         | +1.3%    |
| 21,952   | 56.8      | 59.1        | +4.1%    | 1,449     | 1,531       | +5.6%    |
| 50,653   | 132       | 140         | +6.3%    | 3,377     | 3,599       | +6.6%    |

\* That run's 729-vertex baseline was noisy (p95 1.96 ms against a 1.57 ms median). Three
repeat runs of `--quick --colliders` gave +7.5% to +7.8% (real-time) and +7.1% to +7.7%
(reference) at 729 vertices.

## Reading

- Three colliders cost about 4–8% of frame time, roughly constant across mesh sizes, because
  contact work is one SDF per vertex per collider per iteration, next to about 11 constraint
  projections per vertex. This is within the agreed 10% budget, so no bounding-box pre-check
  was added.
- The cost grows linearly with the number of colliders. Scenes with many colliders would need
  a broad phase (for example a bounding-box test per collider), which is not needed for the
  one-to-three colliders planned for scenarios.
