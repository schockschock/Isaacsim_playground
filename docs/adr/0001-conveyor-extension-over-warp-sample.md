# ADR-0001: Use the conveyor extension, not the Warp sample

## Status
Accepted — 2026-06-29

## Context
The README's conveyor objective links to the `isaacsim.asset.gen.conveyor`
extension docs. Isaac Sim also ships `standalone_examples/conveyor_belt/`, a
9-file sample that implements conveyor physics with custom NVIDIA Warp
kernels (CUDA graphs, contact-point force computation, patch redistribution).

When implementing `simulation_scripts/conveyor.py` we had two viable paths:

1. **The extension** — `create_conveyor_belt(stage, prim)` returns an
   OmniGraph node; speed via a graph variable, direction via an input
   attribute. The extension auto-applies RigidBodyAPI + CollisionAPI +
   PhysxSurfaceVelocityAPI (+ MeshCollisionAPI for meshes). ~10 lines to wire.
2. **The Warp sample** — port `cb_app.py` + 8 supporting modules, manage
   Warp buffers, register physics post-step callbacks, capture CUDA graphs.
   Hundreds of lines, and it duplicates friction logic PhysX already owns.

## Decision
Use the **`isaacsim.asset.gen.conveyor` extension** (`create_conveyor_belt`).
Do not port the `standalone_examples/conveyor_belt/` Warp sample.

## Consequences
- The conveyor extension is not enabled by default in the headless Python
  experience; `simulation_scripts/conveyor.py` loads it explicitly via
  `omni.kit.app.get_extension_manager().set_extension_enabled_immediate(...)`
  before importing `isaacsim.asset.gen.conveyor`. The import must be deferred
  until after the extension is enabled.
- `create_conveyor_belt` only applies CollisionAPI + PhysxSurfaceVelocityAPI
  when the target prim **lacks** RigidBodyAPI. If you pre-apply
  RigidBodyAPI, the belt silently gets no collider and no surface velocity
  (the potato falls through and nothing is transported). Apply kinematic
  **after** calling `create_conveyor_belt`, not before.
- Belt velocity is a graph variable on the action-graph prim
  (`graph:variable:Velocity`), not an attribute on the conveyor node.
  Direction is `inputs:direction` on the node.
- We give up the Warp sample's per-contact friction model and CUDA-graph
  performance. That is fine for a learning playground with one rigid body
  on one belt; revisit if we ever need hundreds of belts or a custom
  friction law.
