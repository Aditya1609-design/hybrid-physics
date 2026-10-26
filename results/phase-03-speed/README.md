# Phase 3 exit check: XPBD against an estimated GNN, per step

Early warning only; the real check is the speed-crossover phase with a trained model and the
final inference path.

- Machine: Apple M1 (4P + 4E cores, 8 GB), macOS 27.0.1. See `machine_info.json`.
- Engine build: `hybrid-physics 0.1.0 | Release | Apple LLVM 21.0.0 (clang-2100.3.34.2) |
  fp-contract=off | sanitizers=none`, commit `805b05a`.
- GNN timing: PyTorch 2.14.1, Python 3.12.3, 4 CPU threads (PyTorch default) and MPS. It was run
  from the same commit plus the then-uncommitted `ml/tools/gnn_cost_estimate.py` (committed
  unchanged as `c98800f`).
- Files: `xpbd_speed.csv` and `gnn_cost.csv` are the raw rows. Reproduce with:
  ```sh
  ./build/release/apps/benchmark/hp_xpbd_speed --csv results/raw/phase-03-speed/xpbd_speed.csv
  .venv/bin/python ml/tools/gnn_cost_estimate.py --budget 2 \
      --xpbd-csv results/raw/phase-03-speed/xpbd_speed.csv \
      --csv-out results/raw/phase-03-speed/gnn_cost.csv
  ```

## Setup

- **XPBD**: unit box (5-tet grid), x = min face pinned, default material, one thread. Frame =
  1/60 s. The real-time preset is 10 substeps × 1 iteration; the reference preset is 256 × 1.
  Median over 200 frames (real-time) or 5–30 frames (reference).
- **GNN**: untrained encode-process-decode, MeshGraphNets layout. 2-hidden-layer MLPs with
  LayerNorm, unshared processor weights, sum aggregation by `index_add_`. Same box graphs as
  XPBD (edge counts checked equal). One forward pass = one simulation step. Median over runs
  within a 2 s budget, after 2 warm-up runs. MPS time includes the final synchronize.
- Variants: `latent × message-passing steps`. `mgn-128x15` is the published MeshGraphNets size.

## XPBD per frame (ms)

| Vertices | Real-time | Reference | ns / constraint projection |
|---------:|----------:|----------:|---------------------------:|
| 729      | 1.55      | 41.2      | 25 |
| 1,331    | 2.91      | 74.5      | 24 |
| 2,197    | 4.94      | 127       | 24 |
| 4,913    | 12.1      | 326       | 26 |
| 10,648   | 27.7      | 739       | 26 |
| 21,952   | 59.3      | 1,483     | 27 |
| 50,653   | 137       | 3,569     | 26 |

Cost is linear in mesh size. The real-time preset fits a 16 ms frame up to about 5k vertices.

## GNN per step: best of CPU and MPS (ms), and ratio to XPBD

| Vertices | mgn-128x15 | 128x10 | 64x10 | 64x5 | 32x3 |
|---------:|-----------:|-------:|------:|-----:|-----:|
| 729      | 52 (33× rt, 1.26× ref) | 35 (23×, 0.86×) | 17.6 (11×, 0.43×) | 9.5 (6.1×, 0.23×) | 6.6 (4.3×, 0.16×) |
| 2,197    | 155 (31×, 1.22×) | 105 (21×, 0.82×) | 51 (10×, 0.40×) | 27 (5.4×, 0.21×) | 10.1 (2.0×, 0.08×) |
| 10,648   | 1,049 (38×, 1.42×) | 763 (28×, 1.03×) | 375 (14×, 0.51×) | 193 (7.0×, 0.26×) | 78 (2.8×, 0.11×) |
| 50,653   | 5,388 (39×, 1.51×) | 3,600 (26×, 1.01×) | 1,891 (14×, 0.53×) | 987 (7.2×, 0.28×) | 343 (2.5×, 0.10×) |

"rt" is GNN time / real-time XPBD time and "ref" is GNN time / reference XPBD time (below 1 means
the GNN is faster). MPS was the faster device everywhere except 32x3 at 50k vertices. Effective
throughput: about 300–400 GFLOP/s on MPS and 140–180 GFLOP/s on CPU, with launch overhead
dominating the small models on small meshes (32x3 never drops below about 6.6 ms on MPS).

## Verdict

1. **Against the real-time preset, a GNN step does not win at any mesh size.** The full
   MeshGraphNets model is 31–40× slower, and the ratio does not shrink as meshes grow, because
   both costs are linear in N. Even the smallest variant (32x3, probably too small to be
   accurate) is 2–4× slower. I see no mesh size where a crossover happens on this hardware.
2. **Against the reference preset, smaller GNNs are faster.** 64x10 takes about half the
   reference cost, 64x5 about a quarter, and the full model is about the same (1.2–1.5× slower).
   So "near-reference accuracy for less than reference cost" is plausible, if a model of that
   size can actually reach reference-level accuracy. That is unknown until GNN training.
3. **A GNN beats real-time XPBD only if one GNN step replaces several frames.** At equal cost
   that is k ≈ 31–40 frames for the full model, about 10–14 for 64x10 and 5–7.5 for 64x5.
   MeshGraphNets-style models are often trained at larger steps than the classical solver, but
   at 1/60 s frames, k > 5 means steps above 80 ms, which is coarse for impacts.

## Why these numbers could move, and in which direction

- XPBD is not optimized: scalar code, one thread, about 25 ns per constraint. SIMD and threads
  (with graph colouring) could plausibly make it 3–8× faster, which makes the GNN look **worse**.
- The GNN numbers are eager PyTorch. Fused kernels, ONNX Runtime, half precision or a
  hand-written kernel could plausibly be 2–5× faster, which makes the GNN look **better**. That
  still would not close a 30× gap for the full model.
- Untrained weights do not change the cost; the input and output sizes are the planned ones.
- On a hybrid run the GNN would cover only part of the mesh, but its cost per vertex stays the
  same, so this does not change the per-vertex comparison.
