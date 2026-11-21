"""Reader for .hptraj trajectory files written by hp_run --record.

The format is described in docs/data_format.md. Only numpy is needed. Frame data is
memory-mapped, so opening a large file is instant and slices like positions[t:t + k] do not
copy anything.

    from data.trajectory import load
    traj = load("run.hptraj")
    x, v = traj.positions, traj.velocities          # [F, N, 3] float32
    x_t, v_t, x_next, v_next = traj.pairs(1)         # single-step training pairs
    edge_index = traj.edge_index()                   # [2, 2E] int64
"""

import json
import os
import warnings

import numpy as np

MAGIC = b"HPTRAJ\0\0"
FORMAT_VERSION = 1
STATUS_OK = 0
STATUS_UNFINISHED = 0xFFFFFFFF

# Fixed 64-byte preamble at the start of every file.
PREAMBLE = np.dtype([
    ("magic", "S8"),
    ("format_version", "<u4"),
    ("header_len", "<u4"),
    ("static_offset", "<u8"),
    ("data_offset", "<u8"),
    ("frame_size", "<u8"),
    ("frames_written", "<u8"),
    ("status", "<u4"),
    ("reserved", "V12"),
])
assert PREAMBLE.itemsize == 64

# Fields every version-1 file must have, with their dtypes, so a damaged or foreign file fails
# with a clear message instead of a confusing numpy error later.
REQUIRED_FRAME_FIELDS = {
    "frame": "<i8", "time": "<f8", "kinetic_j": "<f8", "gravitational_j": "<f8",
    "elastic_j": "<f8", "positions": "<f4", "velocities": "<f4", "solver_owner": "<u1",
}
REQUIRED_STATIC_FIELDS = {
    "rest_positions": "<f4", "inv_mass": "<f4", "region_id": "<i4", "body_id": "<i4",
    "edges": "<i4", "edge_rest_length": "<f4", "edge_compliance": "<f4", "tets": "<i4",
    "tet_rest_volume": "<f4", "tet_compliance": "<f4", "surface_triangles": "<i4",
}


class TrajectoryError(ValueError):
    """The file is not a readable trajectory."""


def _check_fields(table, required, what):
    found = {f["name"]: f["dtype"] for f in table}
    for name, dtype in required.items():
        if name not in found:
            raise TrajectoryError(f"{what} has no field '{name}'")
        if found[name] != dtype:
            raise TrajectoryError(f"{what} field '{name}' is {found[name]}, expected {dtype}")


# First dimensions of each static array, as header count names (the second, if any, is fixed).
STATIC_SHAPES = {
    "rest_positions": ("vertices", 3), "inv_mass": ("vertices",), "region_id": ("vertices",),
    "body_id": ("vertices",), "edges": ("edges", 2), "edge_rest_length": ("edges",),
    "edge_compliance": ("edges",), "tets": ("tets", 4), "tet_rest_volume": ("tets",),
    "tet_compliance": ("tets",), "surface_triangles": ("surface_triangles", 3),
}
FRAME_SHAPES = {"positions": ("vertices", 3), "velocities": ("vertices", 3),
                "solver_owner": ("vertices",)}
INT32_MAX = 2**31 - 1


def _is_count(x):
    return isinstance(x, int) and not isinstance(x, bool) and 0 <= x <= INT32_MAX


def _check_layout(table, expected, counts, what):
    """Rejects negative or non-integer offsets and shapes, and requires the arrays sized by
    the header counts to have exactly those sizes."""
    for f in table:
        if not (isinstance(f["offset"], int) and f["offset"] >= 0):
            raise TrajectoryError(f"{what} field '{f['name']}' has a bad offset")
        if not all(isinstance(d, int) and d >= 0 for d in f["shape"]):
            raise TrajectoryError(f"{what} field '{f['name']}' has a negative or bad shape")
        if f["name"] in expected:
            want = [counts[d] if isinstance(d, str) else d for d in expected[f["name"]]]
            if list(f["shape"]) != want:
                raise TrajectoryError(f"{what} field '{f['name']}' has shape {f['shape']}, "
                                      f"expected {want} from the header counts")


def _layout_dtype(table, itemsize):
    """numpy structured dtype built from a header layout table (unknown fields included)."""
    return np.dtype({
        "names": [f["name"] for f in table],
        "formats": [(f["dtype"], tuple(f["shape"])) for f in table],
        "offsets": [f["offset"] for f in table],
        "itemsize": int(itemsize),
    })


class Trajectory:
    """One recorded run. Frame i is the state after i steps; frame 0 is the initial state."""

    def __init__(self, path, header, preamble, frames, static):
        self.path = path
        self.header = header
        self.status = int(preamble["status"])
        self.frame_size = int(preamble["frame_size"])
        self.frames = frames    # structured array [F], one record per frame
        self.static = static    # dict of static arrays (rest_positions, edges, tets, ...)

    @property
    def closed_cleanly(self):
        return self.status != STATUS_UNFINISHED

    @property
    def num_frames(self):
        return len(self.frames)

    @property
    def num_vertices(self):
        return int(self.header["counts"]["vertices"])

    @property
    def dt(self):
        return float(self.header["dt"])

    @property
    def positions(self):
        return self.frames["positions"]

    @property
    def velocities(self):
        return self.frames["velocities"]

    @property
    def solver_owner(self):
        return self.frames["solver_owner"]

    @property
    def metric_names(self):
        """Per-frame scalar fields (energies, errors, penetration, step time)."""
        return [n for n in self.frames.dtype.names if self.frames.dtype[n].shape == ()]

    def metric(self, name):
        """One per-frame scalar as a [F] array, e.g. metric("kinetic_j")."""
        if name not in self.metric_names:
            raise KeyError(name)
        return self.frames[name]

    def edge_index(self):
        """Directed edges [2, 2E] int64: edge k is i->j and edge k+E is j->i (as hp::MeshGraph)."""
        e = self.static["edges"].astype(np.int64)
        return np.stack([np.concatenate([e[:, 0], e[:, 1]]), np.concatenate([e[:, 1], e[:, 0]])])

    def pairs(self, k=1):
        """Training pairs for a jump of k frames: (x_t, v_t, x_{t+k}, v_{t+k}), each
        [F - k, N, 3]. These are views into the file; nothing is copied."""
        if not 1 <= k < self.num_frames:
            raise ValueError(f"k must be in [1, {self.num_frames - 1}], got {k}")
        x, v = self.positions, self.velocities
        return x[:-k], v[:-k], x[k:], v[k:]


def load(path, warn=True):
    """Opens a trajectory. Raises TrajectoryError if the file is not a valid version-1
    trajectory or is truncated. A file whose writer never finished (crash) is read up to its
    last whole frame, with a warning unless warn=False."""
    path = os.fspath(path)
    file_size = os.path.getsize(path)
    if file_size < PREAMBLE.itemsize:
        raise TrajectoryError(f"{path}: too short to be a trajectory")
    with open(path, "rb") as f:
        raw = f.read(PREAMBLE.itemsize)
        if raw[:8] != MAGIC:
            raise TrajectoryError(f"{path}: not a trajectory file (bad magic)")
        pre = np.frombuffer(raw, dtype=PREAMBLE)[0]
        if int(pre["format_version"]) != FORMAT_VERSION:
            raise TrajectoryError(f"{path}: unsupported format version {int(pre['format_version'])}")
        header_len = int(pre["header_len"])
        static_off, data_off = int(pre["static_offset"]), int(pre["data_offset"])
        frame_size = int(pre["frame_size"])
        if (static_off < PREAMBLE.itemsize + header_len or data_off < static_off
                or frame_size == 0 or data_off > file_size or data_off % 64 or static_off % 64):
            raise TrajectoryError(f"{path}: inconsistent preamble or truncated header")
        try:
            header = json.loads(f.read(header_len).decode("utf-8"))
        except (UnicodeDecodeError, json.JSONDecodeError) as e:
            raise TrajectoryError(f"{path}: bad header: {e}") from None
        f.seek(static_off)
        static_bytes = f.read(data_off - static_off)

    _check_fields(header["frame_layout"], REQUIRED_FRAME_FIELDS, "frame_layout")
    _check_fields(header["static_layout"], REQUIRED_STATIC_FIELDS, "static_layout")
    counts = header["counts"]
    for key in ("vertices", "edges", "tets", "surface_triangles"):
        if not _is_count(counts.get(key)):
            raise TrajectoryError(f"{path}: header count '{key}' is negative or bad")
    _check_layout(header["frame_layout"], FRAME_SHAPES, counts, "frame_layout")
    _check_layout(header["static_layout"], STATIC_SHAPES, counts, "static_layout")
    record = _layout_dtype(header["frame_layout"], frame_size)

    static = {}
    for f in header["static_layout"]:
        dt = np.dtype(f["dtype"])
        count = int(np.prod(f["shape"], dtype=np.int64))
        end = f["offset"] + count * dt.itemsize
        if end > len(static_bytes):
            raise TrajectoryError(f"{path}: static array '{f['name']}' runs past its block")
        static[f["name"]] = np.frombuffer(static_bytes, dtype=dt, count=count,
                                          offset=f["offset"]).reshape(f["shape"])

    whole = (file_size - data_off) // frame_size
    status = int(pre["status"])
    if status == STATUS_UNFINISHED:
        num_frames = whole
        if warn:
            warnings.warn(f"{path}: the run did not finish; reading {whole} whole frames",
                          stacklevel=2)
    else:
        num_frames = int(pre["frames_written"])
        if num_frames > whole:
            raise TrajectoryError(f"{path}: truncated: {whole} frames on disk, "
                                  f"{num_frames} expected")

    if num_frames == 0:
        frames = np.zeros(0, dtype=record)  # np.memmap cannot map zero bytes
    else:
        frames = np.memmap(path, dtype=record, mode="r", offset=data_off, shape=(num_frames,))
    return Trajectory(path, header, pre, frames, static)


def state_hash(positions, velocities):
    """FNV-1a (64-bit) over the raw float32 bytes of positions, then velocities: the same
    'final state hash' hp_run prints."""
    h = 0xCBF29CE484222325
    for arr in (positions, velocities):
        for b in np.ascontiguousarray(arr, dtype="<f4").tobytes():
            h = ((h ^ b) * 0x100000001B3) & 0xFFFFFFFFFFFFFFFF
    return h


def energies(traj, i):
    """Kinetic, gravitational and elastic energy of frame i recomputed from the stored arrays,
    in float64, the same way the engine does (edge offsets are formed in float32 first)."""
    s = traj.static
    x = np.asarray(traj.positions[i])
    v = np.asarray(traj.velocities[i])
    w = s["inv_mass"]
    free = w != 0
    m = 1.0 / w[free].astype(np.float64)
    g = np.array(traj.header["scenario"]["solver"]["gravity"], dtype=np.float64)
    kinetic = 0.5 * np.sum(m * np.sum(v[free].astype(np.float64) ** 2, axis=1))
    gravitational = -np.sum(m * (x[free].astype(np.float64) @ g))

    e = s["edges"]
    d = (x[e[:, 0]] - x[e[:, 1]]).astype(np.float64)
    c = np.linalg.norm(d, axis=1) - s["edge_rest_length"].astype(np.float64)
    a = s["edge_compliance"].astype(np.float64)
    soft = a != 0
    elastic = np.sum(0.5 * c[soft] ** 2 / a[soft])

    t = s["tets"]
    p = x.astype(np.float64)
    pa, pb, pc, pd = p[t[:, 0]], p[t[:, 1]], p[t[:, 2]], p[t[:, 3]]
    vol = np.einsum("ij,ij->i", np.cross(pb - pa, pc - pa), pd - pa) / 6.0
    c = vol - s["tet_rest_volume"].astype(np.float64)
    a = s["tet_compliance"].astype(np.float64)
    soft = a != 0
    elastic += np.sum(0.5 * c[soft] ** 2 / a[soft])
    return float(kinetic), float(gravitational), float(elastic)
