# 0007: Scenario format and loader

Status: proposed (Phase 5)

## Goal

Describe a run as data: one JSON file holds the mesh, material, initial conditions, solver
preset and colliders. The same files serve the example scenes, the headless runner, the viewer
and, later, the randomized dataset generator.

## Decision

- **Two steps.** `parse_scenario`/`load_scenario` produce a plain `Scenario` struct and check
  everything that can be checked without a mesh. `build_scene` creates the mesh, `SimState` and
  `XpbdSolver`, and does the checks that need geometry. Code can also build a `Scenario`
  directly, as the dataset generator will.
- **JSON stays private.** Only `engine/src/scene/scenario.cpp` includes nlohmann/json. The
  engine links it PRIVATE (already on the dependency guard's allowed list), and no public header
  exposes a JSON type.
- **Moduli, not compliances.** Materials are written as `youngs_modulus` and `bulk_modulus` in
  Pa, the units used for the agreed 0.3–30 MPa range, and converted with `compliance = 1/modulus`
  (the definitions in ADR 0005). Absent keys keep `Material`'s defaults bit for bit.
- **A list of bodies.** `bodies` is an array so that multi-body scenes need no format change.
  `build_scene` accepts exactly one body, the current scope.
- **Strict keys.** Unknown keys are errors, with the key's full path in the message. A silently
  ignored typo in a dataset scenario would corrupt the data unnoticed.
- **`format_version`** is required and must be 1, so later changes can be detected.
- **Placement matches the C++ API.** Without a rotation, `position` is passed to the mesh
  generator as its centre, so a scenario gives exactly the same mesh, and the same trajectory
  bit for bit, as the equivalent C++ setup (tested). With a rotation, the mesh is built at the
  origin, rotated in double precision, and moved.
- **Initial velocity** is `v + ω × (x − c)` about the centre of mass `c`, computed in double.
  Pins are applied last, so pinned vertices have zero velocity.
- **`seed`** is required but unused for now: nothing in a version 1 scenario is random.
- **Fixed timestep.** There is no `dt` key; every frame is `kFrameDt`.

### Checks (from the Phase 4 review)

- Reject any vertex, pinned or free, that starts inside a collider. XPBD turns an overlap of
  depth `p` into velocity `p / h`.
- Warn for sphere colliders with a radius under two mean edge lengths (vertex-only contact).
- Warn for sphere or box colliders thin enough to tunnel through. A vertex pushed past a
  collider's middle is projected out through the far side, so the threshold is *half* the
  thickness (sphere radius, smallest box half extent) travelled in one substep. ADR 0006 states
  the condition as the full thickness; half is the correct one. The speed used is
  `max initial speed + |g| · frames · dt`, a loose upper bound.
- Warnings are returned in `Scene::warnings`; the engine has no logger, and the caller decides
  how to show them.

## Alternatives considered

- Compliances in the file: they match the solver directly, but nobody thinks in 1/Pa, and the
  material ranges in the docs are all in moduli.
- Warning on unknown keys instead of failing: too easy to miss in a batch of generated files.
- Includes or inheritance between scenario files: not needed for six examples. Generated
  scenarios will be written whole by the generator.

## Limits

- A perfectly stiff material (zero compliance) cannot be written.
- One body per scene, static colliders only, as in the scope defaults.
- The tunnelling warning uses a loose speed bound and can warn for long, slow runs.
