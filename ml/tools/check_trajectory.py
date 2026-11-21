"""Runs hp_run with --record and checks that the trajectory file agrees with hp_run.

Proves the .hptraj format (docs/data_format.md) is readable from Python and holds exactly what
the engine computed:
  1. frame count and status match the run;
  2. the last frame's FNV-1a hash equals hp_run's printed "final state hash" (bit-exact);
  3. every frame's metrics match hp_run's CSV, to the precision the CSV is printed with;
  4. kinetic, gravitational and elastic energy recomputed in numpy match the recorded values;
  5. graph and training-pair helpers return the expected shapes and data;
  6. a second recording of the same run is identical apart from step times and timestamp;
  7. the plain-numpy example in docs/data_format.md reads the same positions;
  8. copies with a corrupt header (wrapping or negative offsets, negative or oversized counts)
     are rejected with TrajectoryError.

Usage:
  python ml/tools/check_trajectory.py --hp-run build/release/apps/headless_runner/hp_run \
      scenarios/cube_drop.json [--preset reference] [--frames N] [--k 5]
Exit code 0 if every check passes, 1 otherwise.
"""

import argparse
import csv
import json
import os
import re
import subprocess
import sys
import tempfile

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from data.trajectory import TrajectoryError, energies, load, state_hash  # noqa: E402

# Relative tolerance for numpy vs engine energies. Both sum in float64 from the same float32
# data; only the summation order differs.
ENERGY_RTOL = 1e-9

# CSV columns and the precision hp_run prints them with: ("rel", digits) for %.<digits>g,
# ("abs", decimals) for %.<decimals>f, "int" for exact integers. The allowed difference is half
# a unit in the last printed digit.
CSV_COLUMNS = {
    "frame": ("int", None), "time_s": ("abs", 6),
    "kinetic_j": ("rel", 9), "gravitational_j": ("rel", 9), "elastic_j": ("rel", 9),
    "edge_rms": ("rel", 6), "edge_max": ("rel", 6), "volume_rms": ("rel", 6),
    "volume_max": ("rel", 6), "inverted_tets": ("int", None),
    "max_penetration_m": ("rel", 6), "num_penetrating": ("int", None),
    "num_in_contact": ("int", None), "step_ms": ("abs", 4),
}
# CSV column name -> trajectory field name, where they differ.
CSV_TO_FIELD = {"time_s": "time"}


class Checks:
    """Collects PASS/FAIL lines."""

    def __init__(self):
        self.failed = 0

    def report(self, ok, name, detail):
        print(f"{'PASS' if ok else 'FAIL'}  {name}: {detail}")
        self.failed += 0 if ok else 1


def run_hp_run(args, out_dir, name):
    """Runs hp_run once with --record and --csv; returns (trajectory path, csv path, stdout)."""
    traj = os.path.join(out_dir, f"{name}.hptraj")
    csv_path = os.path.join(out_dir, f"{name}.csv")
    cmd = [args.hp_run, args.scenario, "--record", traj, "--csv", csv_path]
    if args.preset:
        cmd += ["--preset", args.preset]
    if args.frames:
        cmd += ["--frames", str(args.frames)]
    proc = subprocess.run(cmd, capture_output=True, text=True)
    if proc.returncode != 0:
        sys.exit(f"hp_run failed with exit code {proc.returncode}:\n{proc.stderr}")
    return traj, csv_path, proc.stdout


def read_csv(path):
    with open(path, newline="") as f:
        rows = [r for r in f if not r.startswith("#")]
    return list(csv.DictReader(rows))


def check_counts(c, traj, rows):
    planned = int(traj.header["frames_planned"])
    ok = (traj.closed_cleanly and traj.status == 0 and traj.num_frames == planned + 1
          and len(rows) == traj.num_frames
          and np.array_equal(traj.metric("frame"), np.arange(traj.num_frames)))
    c.report(ok, "frames and status",
             f"{traj.num_frames} frames (0..{planned}), status {traj.status}, CSV rows {len(rows)}")


def check_hash(c, traj, stdout):
    m = re.search(r"final state hash ([0-9a-f]{16})", stdout)
    printed = m.group(1) if m else "missing"
    computed = f"{state_hash(traj.positions[-1], traj.velocities[-1]):016x}"
    c.report(printed == computed, "final state hash", f"file {computed}, hp_run {printed}")


def check_csv(c, traj, rows):
    worst = {}
    ok = True
    for col, (kind, digits) in CSV_COLUMNS.items():
        rec = np.asarray(traj.metric(CSV_TO_FIELD.get(col, col)), dtype=np.float64)
        csv_vals = np.array([float(r[col]) for r in rows])
        if kind == "int":
            bad = rec != csv_vals
            err = float(np.max(np.abs(rec - csv_vals))) if len(rec) else 0.0
        elif kind == "abs":
            err_arr = np.abs(rec - csv_vals)
            bad = err_arr > 0.5 * 10.0 ** -digits * (1 + 1e-9)
            err = float(err_arr.max())
        else:
            # Relative to the value; values printed as exactly 0 must be 0 in the file too.
            err_arr = np.abs(rec - csv_vals) / np.maximum(np.abs(rec), 1e-300)
            bad = err_arr > 0.5 * 10.0 ** (1 - digits) * (1 + 1e-9)
            err = float(err_arr.max())
        worst[col] = err
        if np.any(bad):
            ok = False
            print(f"      {col}: {int(np.sum(bad))} frames differ beyond print precision")
    c.report(ok, "metrics vs hp_run CSV",
             f"{len(CSV_COLUMNS)} columns x {len(rows)} frames; worst energy rel diff "
             f"{max(worst['kinetic_j'], worst['gravitational_j'], worst['elastic_j']):.1e}")


def check_energies(c, traj):
    names = ("kinetic_j", "gravitational_j", "elastic_j")
    recorded = np.stack([np.asarray(traj.metric(n), dtype=np.float64) for n in names], axis=1)
    worst = 0.0
    ok = True
    for i in range(traj.num_frames):
        mine = np.array(energies(traj, i))
        scale = max(np.sum(np.abs(recorded[i])), 1e-30)  # per-frame energy scale
        rel = float(np.max(np.abs(mine - recorded[i]))) / scale
        worst = max(worst, rel)
        if rel > ENERGY_RTOL:
            ok = False
    c.report(ok, "energies recomputed in numpy",
             f"worst diff {worst:.1e} of the frame's energy scale (limit {ENERGY_RTOL:.0e})")


def check_helpers(c, traj, k):
    n, e = traj.num_vertices, int(traj.header["counts"]["edges"])
    f = traj.num_frames
    ei = traj.edge_index()
    ok_graph = (ei.shape == (2, 2 * e) and ei.dtype == np.int64 and ei.min() >= 0
                and ei.max() < n and np.array_equal(ei[0, :e], ei[1, e:]))
    one = traj.pairs(1)
    jump = traj.pairs(k)
    ok_pairs = (all(a.shape == (f - 1, n, 3) for a in one)
                and all(a.shape == (f - k, n, 3) for a in jump)
                and np.array_equal(jump[2][0], traj.positions[k])
                and np.array_equal(one[3][-1], traj.velocities[-1]))
    c.report(ok_graph and ok_pairs, "graph and pair helpers",
             f"edge_index {ei.shape}, pairs(1) {one[0].shape}, pairs({k}) {jump[0].shape}")


def check_determinism(c, path_a, path_b):
    a, b = load(path_a), load(path_b)
    ha = {k: v for k, v in a.header.items() if k != "created_utc"}
    hb = {k: v for k, v in b.header.items() if k != "created_utc"}
    same_static = all(np.array_equal(a.static[k], b.static[k]) for k in a.static)
    fields = [n for n in a.frames.dtype.names if n != "step_ms"]
    same_frames = a.num_frames == b.num_frames and all(
        np.array_equal(a.frames[n], b.frames[n]) for n in fields)
    c.report(ha == hb and same_static and same_frames, "second recording identical",
             f"header (except created_utc), static block and {len(fields)} frame fields "
             "(except step_ms)")


def check_doc_example(c, path, traj):
    # Same code as the "Reading with numpy" section of docs/data_format.md.
    raw = np.fromfile(path, dtype=np.uint8)
    assert raw[:8].tobytes() == b"HPTRAJ\0\0"
    header_len, = raw[12:16].view("<u4")
    static_off, data_off, frame_size, frames = raw[16:48].view("<u8")
    header = json.loads(raw[64:64 + header_len].tobytes())
    fields = header["frame_layout"]
    rec = np.dtype({"names": [f["name"] for f in fields],
                    "formats": [(f["dtype"], tuple(f["shape"])) for f in fields],
                    "offsets": [f["offset"] for f in fields], "itemsize": int(frame_size)})
    frames_arr = np.memmap(path, dtype=rec, mode="r", offset=int(data_off),
                           shape=(int(frames),))
    x = frames_arr["positions"]
    c.report(np.array_equal(x, traj.positions), "docs/data_format.md numpy example",
             f"positions {x.shape} identical")


def write_with_header(src, dst, edit):
    """Copies src to dst with `edit` applied to the JSON header, keeping its length (compact
    JSON padded with spaces) so every offset in the preamble stays valid."""
    raw = bytearray(open(src, "rb").read())
    header_len = int(np.frombuffer(bytes(raw[12:16]), "<u4")[0])
    header = json.loads(bytes(raw[64:64 + header_len]))
    edit(header)
    text = json.dumps(header, separators=(",", ":")).encode()
    assert len(text) <= header_len
    raw[64:64 + header_len] = text + b" " * (header_len - len(text))
    open(dst, "wb").write(raw)


def check_corrupt_headers(c, path, tmp):
    def set_offset(table, name, value):
        def edit(h):
            for f in h[table]:
                if f["name"] == name:
                    f["offset"] = value
        return edit

    def set_count(count, value, arrays):
        def edit(h):
            h["counts"][count] = value
            for f in h["static_layout"]:
                if f["name"] in arrays:
                    f["shape"][0] = value
            if count == "vertices":
                for f in h["frame_layout"]:
                    if f["shape"]:
                        f["shape"][0] = value
        return edit

    cases = {
        "static offset wrapping 2^64": set_offset("static_layout", "rest_positions", 2**64 - 16),
        "negative static offset": set_offset("static_layout", "edges", -64),
        "negative frame offset": set_offset("frame_layout", "positions", -8),
        "negative edge count": set_count("edges", -1,
                                         {"edges", "edge_rest_length", "edge_compliance"}),
        "negative vertex count": set_count("vertices", -1,
                                           {"rest_positions", "inv_mass", "region_id", "body_id"}),
        "count beyond int32": set_count("tets", 2**40,
                                        {"tets", "tet_rest_volume", "tet_compliance"}),
        "shape disagreeing with counts": set_count("edges", 7, {"edges"}),
    }
    dst = os.path.join(tmp, "corrupt.hptraj")
    write_with_header(path, dst, lambda h: None)
    ok = load(dst) is not None  # the unedited copy must still open
    missed = []
    for name, edit in cases.items():
        write_with_header(path, dst, edit)
        try:
            load(dst)
            missed.append(name)
        except TrajectoryError:
            pass
    c.report(ok and not missed, "corrupt headers rejected",
             f"{len(cases) - len(missed)}/{len(cases)} rejected with TrajectoryError"
             + (f"; accepted: {', '.join(missed)}" if missed else ""))


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--hp-run", required=True, help="path to the hp_run executable")
    ap.add_argument("scenario", help="scenario JSON file")
    ap.add_argument("--preset", choices=["realtime", "reference"])
    ap.add_argument("--frames", type=int, help="override the scenario's frame count")
    ap.add_argument("--k", type=int, default=5, help="jump length for the pair check")
    args = ap.parse_args()

    with tempfile.TemporaryDirectory(prefix="hp_check_") as tmp:
        path_a, csv_a, stdout = run_hp_run(args, tmp, "a")
        path_b, _, _ = run_hp_run(args, tmp, "b")
        traj = load(path_a)
        print(f"{args.scenario}: {traj.num_frames} frames, {traj.num_vertices} vertices, "
              f"{traj.frame_size} bytes per frame, {os.path.getsize(path_a)} bytes, "
              f"preset {traj.header['scenario']['preset']}")
        c = Checks()
        rows = read_csv(csv_a)
        check_counts(c, traj, rows)
        check_hash(c, traj, stdout)
        check_csv(c, traj, rows)
        check_energies(c, traj)
        check_helpers(c, traj, min(args.k, traj.num_frames - 1))
        check_determinism(c, path_a, path_b)
        check_doc_example(c, path_a, traj)
        check_corrupt_headers(c, path_a, tmp)
        del traj  # release the memory map before the directory is removed

    print("ALL CHECKS PASSED" if c.failed == 0 else f"{c.failed} CHECK(S) FAILED")
    return 0 if c.failed == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
