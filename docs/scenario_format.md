# Scenario file format (version 1)

A scenario is one JSON file that fully describes a run: the body, its material and initial
conditions, the solver preset and the colliders. `hp::load_scenario` reads it,
`hp::build_scene` turns it into a mesh, a `SimState` and an `XpbdSolver`, and `hp_run` runs it.
Examples are in `scenarios/`.

Units are SI throughout: metres, seconds, kilograms, pascals, radians per second. Angles in
rotations are in degrees. Vectors are arrays of three numbers. All numbers must fit in float32.

## Rules

- **Unknown keys are errors**, at every level. A typo such as `youngs_modulos` fails with its
  path (`bodies[0].material.youngs_modulos: unknown key`) instead of being silently ignored.
- Every error names the key it is about, e.g. `colliders[1].radius: must be positive`.
- Keys marked *optional* may be left out and take the default shown.
- The timestep is not configurable: every frame is `kFrameDt = 1/60 s`.

## Top level

| Key              | Type          | Required | Meaning |
|------------------|---------------|----------|---------|
| `format_version` | integer       | yes      | must be `1` |
| `name`           | string        | yes      | non-empty; printed by `hp_run` and written to its CSV |
| `seed`           | integer ≥ 0   | yes      | kept with the scenario. Nothing is random yet; randomized scenarios will use it |
| `frames`         | integer ≥ 1   | yes      | number of frames to run |
| `solver`         | object        | optional | see below; default is the real-time preset |
| `bodies`         | array         | yes      | at least one body. `build_scene` currently accepts exactly one |
| `colliders`      | array         | optional | default `[]` |

## `solver`

| Key          | Type        | Default      | Meaning |
|--------------|-------------|--------------|---------|
| `preset`     | string      | `"realtime"` | `"realtime"` (10 substeps × 1 iteration) or `"reference"` (256 × 1) |
| `substeps`   | integer ≥ 1 | from preset  | overrides the preset's substep count |
| `iterations` | integer ≥ 1 | from preset  | overrides the preset's iteration count |
| `gravity`    | vector      | `[0, -9.81, 0]` | m/s² |

## `bodies[i]`

| Key                | Type   | Default          | Meaning |
|--------------------|--------|------------------|---------|
| `mesh`             | object | required         | see below |
| `material`         | object | Material defaults | see below |
| `position`         | vector | `[0, 0, 0]`      | where the mesh centre is placed |
| `rotation`         | rotation | none           | applied about the mesh centre, before `position` |
| `velocity`         | vector | `[0, 0, 0]`      | m/s, added to every vertex |
| `angular_velocity` | vector | `[0, 0, 0]`      | rad/s about the body's centre of mass |
| `pins`             | array of pin boxes | `[]` | vertices to hold fixed |

The rotated, placed mesh is both the initial and the rest shape, so the body starts unstressed.
Each vertex starts with velocity `velocity + angular_velocity × (x − c)`, where `c` is the
centre of mass.

### `mesh`

| `type`     | Keys (all optional, generator defaults shown) |
|------------|-----------------------------------------------|
| `"box"`    | `size` vector `[1, 1, 1]`; `cells` array of 3 integers `[8, 8, 8]` |
| `"beam"`   | `length` `2.0` (along x); `thickness` `0.5`; `cells_along` `20`; `cells_across` `5` |
| `"sphere"` | `radius` `0.5`; `cells_per_diameter` `10` (voxelized, stair-stepped surface) |

Sizes must be positive and cell counts at least 1. Keys of another mesh type are errors.

### `material`

| Key              | Type   | Default | Meaning |
|------------------|--------|---------|---------|
| `density`        | number > 0  | `1000`  | kg/m³ |
| `youngs_modulus` | number > 0  | `1e6`   | Pa; Young's-like modulus of the edge network, `edge_compliance = 1/Y` |
| `bulk_modulus`   | number > 0  | `1e7`   | Pa; bulk-like modulus of the volume constraints, `volume_compliance = 1/K` |
| `damping`        | number ≥ 0  | `0`     | linear velocity damping, 1/s |

The moduli are approximate: see the XPBD solver ADR (0005) for what they mean exactly. A
perfectly stiff material (zero compliance) cannot be written in this format.

### Pin box

`{"min": vector, "max": vector}`, an axis-aligned box in world coordinates of the initial
pose. Every vertex inside it, bounds included, gets zero inverse mass and zero velocity.
`min` must not exceed `max` on any axis, and a box that selects no vertex is an error.

## `colliders[i]`

Every collider has a `type` and an optional `friction` (Coulomb μ ≥ 0, default `0`, the same
for sticking and sliding). Keys of another collider type are errors.

| `type`     | Keys |
|------------|------|
| `"plane"`  | `point` (a point on the plane), `normal` (non-zero, normalized on load, points out of the solid side) |
| `"sphere"` | `center`, `radius` > 0 |
| `"box"`    | `center`, `half_extents` (all > 0), optional `rotation` |

## Rotation

`{"axis": vector, "angle_deg": number}`: a right-handed rotation about `axis` (any non-zero
length) by the angle in degrees. An angle of 0 is exactly the identity.

## Checks when the scene is built

`build_scene` rejects a scenario if:

- it has more than one body;
- a pin box selects no vertex;
- any vertex, pinned or free, starts inside a collider (signed distance < 0). XPBD would turn
  the overlap into a large velocity. Touching (distance exactly 0) is allowed.

It adds a warning, without failing, if:

- a sphere collider's radius is less than twice the mean mesh edge length. Contact is
  vertex-only, so a small sphere can push into the faces between vertices.
- a sphere or box collider is thin enough to tunnel through. The warning fires when a vertex
  could move more than half the collider's thickness (sphere radius, smallest box half extent)
  in one substep, using the speed bound `max initial speed + |g| · frames · dt`. The bound is
  deliberately loose and can warn for long runs where the body never gets that fast.

## Example

```json
{
  "format_version": 1,
  "name": "cube_drop",
  "seed": 1,
  "frames": 180,
  "solver": { "preset": "realtime" },
  "bodies": [
    {
      "mesh": { "type": "box", "size": [0.5, 0.5, 0.5], "cells": [8, 8, 8] },
      "material": { "density": 1000, "youngs_modulus": 1e6, "bulk_modulus": 1e7 },
      "position": [0, 1.0, 0]
    }
  ],
  "colliders": [
    { "type": "plane", "point": [0, 0, 0], "normal": [0, 1, 0], "friction": 0.5 }
  ]
}
```
