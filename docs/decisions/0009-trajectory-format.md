# 0009: Trajectory recording format

Status: proposed (Phase 7)

## Goal

Record every frame of a run (positions, velocities, solver owner, metrics) with enough context
to reproduce it (scenario, seed, solver settings, engine commit, machine), in a file Python can
use to train a GNN on single steps (t → t+1) and on multi-frame jumps (t → t+k).

## Decision

- **Own binary format, `.hptraj`**, specified in `docs/data_format.md`: a 64-byte preamble, a
  JSON header, a static mesh block, then fixed-size frame records, all little-endian.
- **Fixed-size frame records.** A record holds the scalar metrics followed by positions,
  velocities and solver owner for every vertex, so record i sits at
  `data_offset + i * frame_size`. In numpy that is one `np.memmap` with a structured dtype:
  `positions[t]` and `positions[t + k]` are views, and opening a file reads nothing but the
  header. Any jump length k comes from the same file.
- **Streaming writes, patched on close.** Records are appended during the run.
  `frames_written` and `status` are written as "unfinished" first and patched in the preamble
  when the run ends, so a crash leaves a readable file of whole frames that is clearly marked
  incomplete.
- **Self-describing layout.** The header lists every static array and frame field with its
  numpy dtype, shape and offset. Readers look fields up by name, so fields can be added later
  without breaking old readers (see Versioning in the format spec).
- **Run context in the header.** The scenario file text, the settings actually used after
  overrides, colliders, seed, `build_info`, git commit and dirty flag (generated on every build
  by `cmake/GitInfo.cmake`), machine description, and the elastic-energy caveat.
- **The engine stays clock-free and machine-agnostic.** `IRecorder`/`TrajectoryWriter` live in
  the engine's `io` module; the app passes in the step times, timestamp and machine info
  (`hp_run` collects them, with the Apple chip name read behind `#ifdef __APPLE__` in the app).
- **The static block holds the constraint data** (edges, rest lengths, compliances, tets, rest
  volumes), so Python can rebuild the graph and recompute every energy without the engine.

## Alternatives considered

- **HDF5:** chunking, compression and wide tool support, but a large C dependency for the engine
  and `h5py` for Python, and crash safety needs care.
- **`.npz` per run:** trivial in Python, but it cannot be streamed (the whole run would sit in
  memory until the end), and it needs a zip writer in C++.
- **One file per frame:** simple to write, but thousands of small files per run, and loading a
  window of frames costs many opens.
- **Separate position and velocity blocks for the whole run** (instead of one record per frame):
  contiguous `[F, N, 3]` arrays, but the frame count must be known in advance or the file
  rewritten at the end, which breaks crash tolerance.

## Consequences and limits

- About `25 N + 112` bytes per frame: 18,344 bytes for the 729-vertex cube, about 11 MB for 600
  frames. No compression: it would break memory mapping. Phase 10 can compress whole files for
  storage if the dataset gets large.
- Field views from a memmap are strided (one record per frame), not contiguous. Copy a window
  with `np.ascontiguousarray` before heavy use, or when moving it to a tensor.
- Little-endian only; the writer refuses big-endian hosts. Both development machines are arm64.
- `step_ms` and `created_utc` are not deterministic; everything else in two recordings of the
  same run with the same build is byte-identical (tested).
- One file holds one run of one scene. Multi-body scenes fit as they are (`body_id` per vertex);
  a changing vertex count would need a new format version.
