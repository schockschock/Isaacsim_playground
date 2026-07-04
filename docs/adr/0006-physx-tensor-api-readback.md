# ADR-0006: PhysX Tensor API readback via RigidPrim

## Status
Accepted — 2026-07-04

## Context
The dataset requires the PhysX solver's true internal state (position, orientation,
velocity, mass, inertia) at each frame. There are two ways to read this state:

1. **USD-attribute readback** — read `xformOp:translate`, `physics:velocity`,
   etc. from the USD prim after each step. These are PhysX's output written back
   to the USD stage, but involve an extra USD<->PhysX bridge and are only the
   final converged values after constraint solving.
2. **Tensor API** — `omni.physics.tensors` with `RigidPrim` (warp frontend)
   reads the solver's internal buffer directly. No USD indirection, and extends
   to mass/inertia/COM which have no USD attribute equivalents.

Additionally, the angular velocity in USD attributes is stored in **degrees per
second** (`physics:angularVelocity`) while the Tensor API reports **radians per
second**. Mixing the two would silently corrupt downstream energy/momentum
computations.

## Decision
Use the PhysX Tensor API via `isaacsim.core.experimental.prims.RigidPrim` for
all state readback. The module `common.readback.py` encapsulates the setup:

```python
rp = setup_rigid_body_view("/World/Potato")
state = read_state(rp)
```

`RigidPrim` is backed by `SimulationManager.initialize_physics()` which must be
called before creating views. This creates a warp-frontend simulation view that
reads directly from the PhysX solver's tensor buffers.

The old `common.asset.read_rigid_body_state()` is preserved for settling
detection in `loop.py` (which only needs position) and backward compatibility.

### Why RigidPrim over raw tensor API

The low-level `omni.physics.tensors.create_simulation_view` + `.create_rigid_body_view`
API requires manual stage_id handling, subspace roots, and proper ordering
relative to the timeline. `RigidPrim` (from `isaacsim.core.experimental.prims`)
handles all of this via the `SimulationManager` lifecycle and provides a clean,
stable interface already used throughout Isaac Sim's own internals.

The RigidPrim also provides `get_masses()`, `get_coms()`, and `get_inertias()` —
all required by Slice B's energy/momentum pipeline.

## Consequences
- `common/readback.py` depends on `isaacsim.core` extensions (SimulationManager,
  experimental prims). It is **not** pure-logic and cannot run under stdlib unittest.
- All state values are in SI units (m, m/s, rad/s, kg) as reported by the
  Tensor API — no deg/s conversion needed.
- The existing `asset.py:read_rigid_body_state()` continues to work for settling
  detection and backward compatibility with scripts not yet migrated.
- Quaternion convention is wxyz (matching RigidPrim.get_world_poses()).
