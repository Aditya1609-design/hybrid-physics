# Trajectory format (`.hptraj`, version 1)

One file holds one simulation run: every frame's positions, velocities, solver owner and
metrics, plus everything needed to reproduce and interpret the run (scenario, seed, solver
settings, engine commit, machine). It is written by `hp::TrajectoryWriter`
(`engine/include/hp/io/trajectory.hpp`) and read by `hp::TrajectoryReader` in C++.

Design goals: Python can read it with numpy alone, frames stream to disk during the run, a
crashed run is still readable, and the frame data maps straight onto `np.memmap` without
copying. The reasons are in decision 0009.

## File layout

All integers and floats are **little-endian**. Offsets are in bytes from the start of the file
unless stated otherwise.

```
+-----------------------------+  0
| preamble (64 bytes)         |
+-----------------------------+  64
| header JSON (UTF-8)         |  header_len bytes
| zero padding                |  up to a multiple of 64
+-----------------------------+  static_offset
| static block                |  mesh and material arrays, once
+-----------------------------+  data_offset
| frame record 0              |  frame_size bytes each
| frame record 1              |
| ...                         |
+-----------------------------+
```

### Preamble

| Offset | Type      | Field            | Meaning |
|--------|-----------|------------------|---------|
| 0      | char[8]   | magic            | `HPTRAJ\0\0` |
| 8      | u32       | format_version   | `1` |
| 12     | u32       | header_len       | length of the header JSON in bytes |
| 16     | u64       | static_offset    | start of the static block (multiple of 64) |
| 24     | u64       | data_offset      | start of frame record 0 (multiple of 64) |
| 32     | u64       | frame_size       | bytes per frame record (multiple of 8) |
| 40     | u64       | frames_written   | number of frame records; rewritten when the run ends |
| 48     | u32       | status           | `0` ok; `0xFFFFFFFF` never closed; anything else is the producer's error code |
| 52     | 12 bytes  | reserved         | zero |

`frames_written` and `status` are written as 0 and `0xFFFFFFFF` when the file is created and
patched in place when the run ends. `hp_run` stores its exit code as the status, so `3` means the
state became non-finite and the run stopped early (the frames before that are valid).

**Unfinished files.** If `status` is `0xFFFFFFFF` the producer crashed or was killed. Readers
then count whole records, `(file_size - data_offset) // frame_size`, ignore a partial last
record, and warn. If `status` is anything else and the file holds fewer than `frames_written`
records, the file is truncated and readers reject it.

### Header JSON

A UTF-8 JSON object. Readers must ignore keys they do not know. Keys in version 1:

| Key | Content |
|-----|---------|
| `format`, `format_version`, `endianness` | `"hptraj"`, `1`, `"little"` |
| `dt` | frame timestep in seconds (the float32 value of 1/60, written as a double) |
| `frames_planned` | frames the run was asked for (frame 0 not counted) |
| `counts` | `vertices` N, `edges` E, `tets` T, `surface_triangles` S |
| `scenario` | `path`, `name`, `seed`, `preset`, `frames`, `solver` (`substeps`, `iterations`, `gravity`), `colliders`, and `text`: the scenario file exactly as read |
| `engine` | `version`, `build_info` (compiler, flags, sanitizers), `git_commit`, `git_dirty` |
| `machine` | key/value strings describing the machine (same keys as `scripts/machine_info.sh`) |
| `created_utc` | when the file was written (metadata only) |
| `notes` | plain-language reminders, including the elastic-energy caveat below |
| `static_layout` | table of the static block arrays (see below) |
| `frame_layout` | table of the frame record fields (see below) |
| `frame_size` | same as the preamble |

`scenario.solver` and `scenario.frames` are the values actually used, after command-line
overrides such as `hp_run --preset reference`. `scenario.text` is the unmodified file, so the two
may differ; `scenario.preset` says which preset was applied.

`scenario.colliders` lists each collider with only the parameters of its kind:
plane `point`, `normal`; sphere `center`, `radius`; box `center`, `half_extents`, `rotation`
(3x3, row-major, columns are the box axes); all have `friction`.

`git_dirty` is true when tracked files differed from `git_commit` **at build time**; untracked
files are ignored. It does not see edits made after the binary was built.

### Layout tables

`static_layout` and `frame_layout` are arrays of
`{"name": ..., "dtype": ..., "shape": [...], "offset": ...}`. `dtype` uses numpy notation
(`<f4`, `<f8`, `<i4`, `<i8`, `<u1`). Static offsets are relative to `static_offset`; frame offsets
are relative to the start of each record. Readers should build their types from these tables
instead of hard-coding offsets; the tables below are what version 1 writes.

Readers must not trust these tables blindly. Both readers reject a file whose `counts` are
negative or above 2^31 - 1, whose offsets or shape dimensions are negative, whose arrays sized by
a count have a different shape, or whose field would end past its block. The bounds check is
written as `offset > limit || size > limit - offset` so a huge offset cannot wrap around.

### Static block

Each array starts on a 64-byte boundary inside the block.

| Name | dtype | Shape | Meaning |
|------|-------|-------|---------|
| `rest_positions` | `<f4` | [N, 3] | rest shape in the initial pose, metres |
| `inv_mass` | `<f4` | [N] | 1/kg; 0 means pinned |
| `region_id` | `<i4` | [N] | mesh region (all 0 until regions exist) |
| `body_id` | `<i4` | [N] | body index of each vertex |
| `edges` | `<i4` | [E, 2] | undirected edges, `i < j`, sorted |
| `edge_rest_length` | `<f4` | [E] | metres |
| `edge_compliance` | `<f4` | [E] | XPBD compliance alpha of each edge constraint (0 = rigid) |
| `tets` | `<i4` | [T, 4] | tetrahedra, positively oriented |
| `tet_rest_volume` | `<f4` | [T] | m^3 |
| `tet_compliance` | `<f4` | [T] | XPBD compliance alpha of each volume constraint (0 = rigid) |
| `surface_triangles` | `<i4` | [S, 3] | boundary faces, counter-clockwise seen from outside |

**Graph edges.** The file stores each undirected edge once. The GNN graph (`hp::MeshGraph`) uses
two directed edges per undirected edge: directed edge `k` is `edges[k,0] -> edges[k,1]` and
directed edge `k + E` is the reverse. As PyTorch's `edge_index` [2, 2E]:

```python
senders   = np.concatenate([edges[:, 0], edges[:, 1]])
receivers = np.concatenate([edges[:, 1], edges[:, 0]])
```

### Frame record

Record `i` is frame `i`. **Frame 0 is the initial state, before any step**, and frame `i` is the
state after `i` steps of `dt`. Records are written in order with no gaps.

| Offset | Name | dtype | Shape | Unit / meaning |
|--------|------|-------|-------|----------------|
| 0   | `frame` | `<i8` | | frame number (equals the record index) |
| 8   | `time` | `<f8` | | `frame * dt`, seconds |
| 16  | `kinetic_j` | `<f8` | | kinetic energy, J |
| 24  | `gravitational_j` | `<f8` | | gravitational potential energy `-m g.x`, J, zero at the origin |
| 32  | `elastic_j` | `<f8` | | `sum C^2 / (2 alpha)`, J (see the caveat below) |
| 40  | `edge_rms` | `<f8` | | RMS relative edge length error `|l - L| / L` |
| 48  | `edge_max` | `<f8` | | max relative edge length error |
| 56  | `volume_rms` | `<f8` | | RMS relative volume error `|V - V0| / V0` |
| 64  | `volume_max` | `<f8` | | max relative volume error |
| 72  | `max_penetration_m` | `<f8` | | deepest vertex penetration into a collider, m |
| 80  | `rms_penetration_m` | `<f8` | | RMS depth over penetrating vertices, m (0 if none) |
| 88  | `step_ms` | `<f8` | | wall time of the step that produced this frame, ms (0 for frame 0) |
| 96  | `inverted_tets` | `<i4` | | tets with `V <= 0` |
| 100 | `num_penetrating` | `<i4` | | free vertices with signed distance < 0 |
| 104 | `num_in_contact` | `<i4` | | free vertices within 1e-4 m of a collider |
| 108 | (padding) | | 4 bytes | zero |
| 112 | `positions` | `<f4` | [N, 3] | metres |
| 112 + 12N | `velocities` | `<f4` | [N, 3] | m/s |
| 112 + 24N | `solver_owner` | `<u1` | [N] | 0 = XPBD, 1 = GNN (all 0 until the hybrid phases) |

`frame_size = ceil((112 + 25 N) / 8) * 8`; trailing padding is zero. For the 729-vertex
`cube_drop` mesh that is 18,344 bytes per frame (about 18 KB), so 600 frames take about 11 MB.

Energies and errors are accumulated in double from the float32 state. Pinned vertices are left
out of the kinetic and gravitational terms and of the penetration counts; rigid
(zero-compliance) constraints are left out of the elastic term.

## Elastic energy is only meaningful near convergence

`elastic_j` evaluates the XPBD constraint energy `C^2 / (2 alpha)` of the stored positions. It is
physical strain energy only when the solver has (nearly) converged: the **reference preset**, or
a body at rest. The **realtime preset** leaves the constraints unconverged after its few
iterations, and the value then mostly measures that leftover residual: `cantilever_beam` on the
realtime preset reaches about 2.6 kJ of "elastic energy" at t = 1 s, with no impact, although
the run starts with only 1.2 kJ in total. Do not add `elastic_j` to the other terms to
check energy conservation on realtime runs, and do not use it as a training target from them.
Kinetic and gravitational energy are exact for any preset.

## Determinism

Positions and velocities are bit-exact copies of the engine state, and the engine is
deterministic for a given build and machine. Two recordings of the same scenario with the same
build are byte-identical except for:

- `step_ms` in every frame record (wall-clock timing), and
- `created_utc` and `machine` in the header.

`hp_run` prints a `final state hash`: FNV-1a (64-bit, offset basis `0xcbf29ce484222325`,
prime `0x100000001b3`) over the raw bytes of the last frame's `positions`, then its `velocities`.
The same hash can be computed from a recorded frame to prove the file holds exactly the state
the engine computed.

## Training pairs

Every frame is stored, so the same file serves single-step and multi-frame training:

- single step: input frame `t`, target frame `t + 1`;
- jump of `k` frames: input frame `t`, target frame `t + k`, for `t = 0 .. F - k`
  (`F` = last frame index).

## Producing and reading files

```sh
./build/release/apps/headless_runner/hp_run scenarios/cube_drop.json --record run.hptraj
.venv/bin/python ml/tools/check_trajectory.py \
    --hp-run build/release/apps/headless_runner/hp_run scenarios/cube_drop.json
```

`ml/data/trajectory.py` is the Python reader (numpy only). `load(path)` validates the file and
returns a `Trajectory` with memory-mapped `positions`, `velocities` and `solver_owner`
([F, N, 3] / [F, N]), `metric(name)` for the per-frame scalars, `static` (the static arrays),
`header`, `edge_index()` and `pairs(k)`. `state_hash` and `energies` recompute the hash and the
energies from the stored arrays. `check_trajectory.py` runs hp_run, records, and checks the file
against hp_run's printed hash and CSV, the recomputed energies, a second recording, and the
example below; `scripts/ci.sh` runs it when `.venv` has numpy.

## Reading with numpy

```python
import json, numpy as np

raw = np.fromfile("run.hptraj", dtype=np.uint8)
assert raw[:8].tobytes() == b"HPTRAJ\0\0"
header_len, = raw[12:16].view("<u4")
static_off, data_off, frame_size, frames = raw[16:48].view("<u8")
header = json.loads(raw[64:64 + header_len].tobytes())

fields = header["frame_layout"]
rec = np.dtype({"names": [f["name"] for f in fields],
                "formats": [(f["dtype"], tuple(f["shape"])) for f in fields],
                "offsets": [f["offset"] for f in fields], "itemsize": int(frame_size)})
frames_arr = np.memmap("run.hptraj", dtype=rec, mode="r", offset=int(data_off), shape=(int(frames),))
x = frames_arr["positions"]          # [F+1, N, 3] float32
```

## Versioning

- Adding header keys, or appending fields to a layout table, keeps version 1: readers look
  fields up by name and ignore unknown ones.
- Changing the meaning, type or shape of an existing field, the preamble, or the alignment rules
  bumps `format_version`. Readers reject versions they do not know.
