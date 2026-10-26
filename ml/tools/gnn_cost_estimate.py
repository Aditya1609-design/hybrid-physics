"""Per-step cost estimate for a MeshGraphNets-style GNN on our box meshes.

Two estimates, both for an untrained model (weights do not change the cost):
  1. Analytic FLOP and memory-traffic count, no dependencies beyond the standard library.
  2. Wall-clock timing in PyTorch on CPU and MPS, with message passing written as plain
     index_add_ (no PyG), the way the training code will do it.

Usage:
  python ml/tools/gnn_cost_estimate.py                      # FLOPs + CPU/MPS timing
  python ml/tools/gnn_cost_estimate.py --flops-only         # no PyTorch needed
  python ml/tools/gnn_cost_estimate.py --xpbd-csv results/raw/phase-03-speed/xpbd_speed.csv
"""

import argparse
import csv
import statistics
import sys
import time

# Box cells per side; matches apps/benchmark/xpbd_speed.cpp (729 to 50653 vertices).
DEFAULT_CELLS = [8, 10, 12, 16, 21, 27, 36]

# (name, latent size, message-passing steps). "mgn" is the published MeshGraphNets setup.
VARIANTS = [
    ("mgn-128x15", 128, 15),
    ("128x10", 128, 10),
    ("64x10", 64, 10),
    ("64x5", 64, 5),
    ("32x3", 32, 3),
]

NODE_IN = 5   # velocity (3) + one-hot node type (2), as in hp::node_features
EDGE_IN = 8   # current offset (3) + length (1) + rest offset (3) + rest length (1)
OUT = 3       # predicted acceleration or velocity change per node
MLP_HIDDEN_LAYERS = 2


def box_counts(cells):
    """Vertex and undirected edge counts of the 5-tet box grid with cells^3 cells."""
    n = cells + 1
    axis_edges = 3 * cells * n * n
    face_diagonals = 3 * cells * cells * n  # one diagonal per cell face
    return n ** 3, axis_edges + face_diagonals


def box_edges(cells):
    """Undirected edge list of the box grid (axis edges plus one diagonal per face)."""
    n = cells + 1

    def vid(i, j, k):
        return (k * n + j) * n + i

    edges = []
    for k in range(n):
        for j in range(n):
            for i in range(n):
                if i < cells:
                    edges.append((vid(i, j, k), vid(i + 1, j, k)))
                if j < cells:
                    edges.append((vid(i, j, k), vid(i, j + 1, k)))
                if k < cells:
                    edges.append((vid(i, j, k), vid(i, j, k + 1)))
    # Face diagonals alternate with cell parity, as in the alternating 5-tet split.
    for k in range(n):
        for j in range(n):
            for i in range(n):
                odd = (i + j + k) % 2
                if i < cells and j < cells:
                    edges.append((vid(i + odd, j, k), vid(i + 1 - odd, j + 1, k)))
                if i < cells and k < cells:
                    edges.append((vid(i + odd, j, k), vid(i + 1 - odd, j, k + 1)))
                if j < cells and k < cells:
                    edges.append((vid(i, j + odd, k), vid(i, j + 1 - odd, k + 1)))
    return edges


def mlp_flops(d_in, latent, d_out, layer_norm):
    """Multiply-adds count as 2 FLOPs; LayerNorm is about 8 FLOPs per output value."""
    dims = [d_in] + [latent] * MLP_HIDDEN_LAYERS + [d_out]
    flops = sum(2 * a * b for a, b in zip(dims[:-1], dims[1:]))
    return flops + (8 * d_out if layer_norm else 0)


def analytic_cost(num_nodes, num_directed_edges, latent, steps):
    """FLOPs and approximate DRAM bytes for one forward pass (float32)."""
    n, e, L = num_nodes, num_directed_edges, latent
    flops = n * mlp_flops(NODE_IN, L, L, True) + e * mlp_flops(EDGE_IN, L, L, True)
    per_step = e * mlp_flops(3 * L, L, L, True) + n * mlp_flops(2 * L, L, L, True)
    per_step += e * L + e * L + n * L  # sum aggregation and the two residual adds
    flops += steps * per_step + n * mlp_flops(L, L, OUT, False)

    # Per step: read edge latents and the gathered sender/receiver latents, write new edge
    # latents, scatter-add into nodes, read/write node latents. Intermediate MLP activations
    # are counted once each way.
    f = 4
    step_bytes = f * (e * L * 3 + e * L + e * L * 2 * MLP_HIDDEN_LAYERS + e * L
                      + n * L * 4 + n * L * 2 * MLP_HIDDEN_LAYERS)
    bytes_total = steps * step_bytes + f * (n * (NODE_IN + L) + e * (EDGE_IN + L) + n * OUT)
    return flops, bytes_total


def build_model(torch, latent, steps):
    """Encode-process-decode GNN with unshared processor weights, as in MeshGraphNets."""
    nn = torch.nn

    def mlp(d_in, d_out, layer_norm=True):
        layers = [nn.Linear(d_in, latent), nn.ReLU()]
        for _ in range(MLP_HIDDEN_LAYERS - 1):
            layers += [nn.Linear(latent, latent), nn.ReLU()]
        layers.append(nn.Linear(latent, d_out))
        if layer_norm:
            layers.append(nn.LayerNorm(d_out))
        return nn.Sequential(*layers)

    class Gnn(nn.Module):
        def __init__(self):
            super().__init__()
            self.node_enc = mlp(NODE_IN, latent)
            self.edge_enc = mlp(EDGE_IN, latent)
            self.edge_mlps = nn.ModuleList(mlp(3 * latent, latent) for _ in range(steps))
            self.node_mlps = nn.ModuleList(mlp(2 * latent, latent) for _ in range(steps))
            self.dec = mlp(latent, OUT, layer_norm=False)

        def forward(self, x, e, senders, receivers):
            h = self.node_enc(x)
            m = self.edge_enc(e)
            for edge_mlp, node_mlp in zip(self.edge_mlps, self.node_mlps):
                m = m + edge_mlp(torch.cat([m, h[senders], h[receivers]], dim=1))
                agg = torch.zeros_like(h).index_add_(0, receivers, m)
                h = h + node_mlp(torch.cat([h, agg], dim=1))
            return self.dec(h)

    return Gnn()


def time_model(torch, device, cells, latent, steps, budget_s, min_runs=3, max_runs=50):
    """Median forward time in ms on random inputs with the real graph structure."""
    und = box_edges(cells)
    n = (cells + 1) ** 3
    src = [a for a, b in und] + [b for a, b in und]  # forward edges, then reverses
    dst = [b for a, b in und] + [a for a, b in und]
    senders = torch.tensor(src, dtype=torch.int64, device=device)
    receivers = torch.tensor(dst, dtype=torch.int64, device=device)
    gen = torch.Generator().manual_seed(0)
    x = torch.randn(n, NODE_IN, generator=gen).to(device)
    e = torch.randn(len(src), EDGE_IN, generator=gen).to(device)

    torch.manual_seed(0)
    model = build_model(torch, latent, steps).to(device).eval()

    def sync():
        if device == "mps":
            torch.mps.synchronize()

    times = []
    with torch.inference_mode():
        for _ in range(2):  # warm-up: kernel compilation and allocator setup
            model(x, e, senders, receivers)
        sync()
        start = time.perf_counter()
        while len(times) < max_runs:
            t0 = time.perf_counter()
            model(x, e, senders, receivers)
            sync()
            times.append(1000.0 * (time.perf_counter() - t0))
            if len(times) >= min_runs and time.perf_counter() - start > budget_s:
                break
    del model, x, e, senders, receivers
    if device == "mps":
        torch.mps.empty_cache()
    return statistics.median(times), len(times)


def read_xpbd(path):
    """{(vertices, preset): median_ms} from the hp_xpbd_speed CSV."""
    out = {}
    with open(path) as f:
        rows = (line for line in f if not line.startswith("#"))
        for r in csv.DictReader(rows):
            out[(int(r["vertices"]), r["preset"])] = float(r["median_ms"])
    return out


def main():
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument("--cells", type=int, nargs="+", default=DEFAULT_CELLS)
    p.add_argument("--devices", nargs="+", default=["cpu", "mps"])
    p.add_argument("--flops-only", action="store_true")
    p.add_argument("--budget", type=float, default=3.0, help="seconds of timing per config")
    p.add_argument("--max-gflop", type=float, default=2000.0,
                   help="skip timing configs above this many GFLOP per step")
    p.add_argument("--xpbd-csv", help="hp_xpbd_speed CSV to compare against")
    p.add_argument("--csv-out", help="write all rows to this CSV")
    args = p.parse_args()

    torch = None
    if not args.flops_only:
        try:
            import torch  # noqa: F811
        except ImportError:
            print("PyTorch not installed; falling back to --flops-only", file=sys.stderr)
            args.flops_only = True
    if torch is not None:
        devices = [d for d in args.devices if d != "mps" or torch.backends.mps.is_available()]
        print(f"torch {torch.__version__}, cpu threads {torch.get_num_threads()}, "
              f"devices {devices}")
    else:
        devices = []

    xpbd = read_xpbd(args.xpbd_csv) if args.xpbd_csv else {}
    rows = []
    header = (f"{'verts':>7} {'dir_edges':>9} {'variant':>11} {'GFLOP':>8} {'MB':>8} "
              + "".join(f"{d + '_ms':>10}" for d in devices)
              + (f"{'xpbd_rt':>9} {'xpbd_ref':>9} {'best/rt':>8} {'best/ref':>9}" if xpbd else ""))
    print(header)
    for cells in args.cells:
        n, und = box_counts(cells)
        e = 2 * und
        for name, latent, steps in VARIANTS:
            flops, nbytes = analytic_cost(n, e, latent, steps)
            row = {"vertices": n, "directed_edges": e, "variant": name, "latent": latent,
                   "steps": steps, "gflop": flops / 1e9, "mbytes": nbytes / 1e6}
            line = f"{n:>7} {e:>9} {name:>11} {flops / 1e9:>8.2f} {nbytes / 1e6:>8.1f} "
            best = None
            for d in devices:
                if flops / 1e9 > args.max_gflop:
                    row[f"{d}_ms"] = ""
                    line += f"{'skip':>10}"
                    continue
                ms, runs = time_model(torch, d, cells, latent, steps, args.budget)
                row[f"{d}_ms"] = round(ms, 3)
                best = ms if best is None else min(best, ms)
                line += f"{ms:>10.2f}"
            rt, ref = xpbd.get((n, "realtime")), xpbd.get((n, "reference"))
            if xpbd:
                row.update(xpbd_realtime_ms=rt, xpbd_reference_ms=ref)
                ratio = lambda a, b: f"{a / b:.2f}" if a and b else "-"  # noqa: E731
                line += (f"{rt or 0:>9.2f} {ref or 0:>9.2f} "
                         f"{ratio(best, rt):>8} {ratio(best, ref):>9}")
            print(line, flush=True)
            rows.append(row)

    if args.csv_out:
        keys = sorted({k for r in rows for k in r}, key=lambda k: list(rows[0]).index(k)
                      if k in rows[0] else len(rows[0]))
        with open(args.csv_out, "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=keys)
            w.writeheader()
            w.writerows(rows)


if __name__ == "__main__":
    main()
