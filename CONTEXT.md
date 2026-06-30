# CONTEXT.md

## Project context
IsaacSim Playground — a personal learning lab for Isaac Sim 6.0.1, PhysX, and Omniverse Kit.
The domain is real-time physics simulation + synthetic data generation: authoring USD scenes,
simulating rigid-body dynamics, and rendering headless observations of one complex solid
(the "potato") across four canonical scenarios.

## Canonical scenarios (the unit of work; refer to issues/ADRs by these names)
Each is a standalone script producing PNG frames + an MP4 of a headless PathTraced sim.

| Scenario    | Script                                  | Definition                                                              | Camera                                  | Key physics                                            |
|-------------|-----------------------------------------|-------------------------------------------------------------------------|-----------------------------------------|--------------------------------------------------------|
| freefall    | samples/simulations/freefall.py         | Potato dropped onto the gridroom ground from rest with initial spin.    | static, looking at impact zone          | gravity + contact + restitution/friction               |
| colliders   | samples/simulations/colliders.py        | Three potato prims, one per collider approximation, dropped side-by-side.| static, side view                       | freefall ×3, varying MeshCollisionAPI approximation    |
| object_throw| samples/simulations/object_throw.py     | Potato launched from the side, rebounds off a vertical wall then ground.| static, wide framing over full trajectory | ballistic flight + wall impact + floor impact         |
| conveyor    | samples/simulations/conveyor.py         | Potato transported by a conveyor belt then falling off the end.         | static, framing belt + drop zone        | kinematic belt (PhysxSurfaceVelocityAPI) + friction + fall-off |

## Glossary (use these terms exactly)
- **Potato** — the project's stand-in complex solid. USD at `/data2/adrien/clean_dataset/2R1-1/2R1-1_centered.usd` (centered mesh + textures). A scanned irregular rigid body, not a real potato; used in every scenario.
- **Stage** — USD scene graph; built from the `gridroom` template (`common/world.py:setup_stage`).
- **Prim** — path-addressed USD object, e.g. `/World/Potato`.
- **SimulationApp** — Isaac Sim entrypoint; must be instantiated headless before any isaacsim/omni/pxr import (`common/app.py:launch`).
- **Replicator** — Isaac Sim SDG layer: camera, render product, BasicWriter (`common/replicator.py`).
- **Render product** — camera output at a resolution; snapshots renderer config at creation (apply renderer settings FIRST).
- **PathTracing** — mandated render mode; per-frame independent, avoids RealTime/TAA ghosting on fast-moving bodies.
- **Rigid body** — prim with UsdPhysics.RigidBodyAPI; dynamic for the potato, kinematic for conveyor belts.
- **Collider** — collision geometry; three APIs per mesh: UsdPhysics.CollisionAPI + PhysxSchema.PhysxCollisionAPI + UsdPhysics.MeshCollisionAPI (`common/asset.py:add_colliders`).
- **Collider approximation** — PhysX mesh-collider representation. Project compares `convexHull` (fast/coarse), `convexDecomposition` (multi-convex), `sdf` (signed distance field, high fidelity). Set via MeshCollisionAPI `approximation`.
- **Physics material** — restitution + friction, bound via `material:binding:physics` (NOT `material:binding`, which is for shaders) (`common/asset.py:apply_physics_material`).
- **PhysX rates** — timeline 120 fps × solver 480 Hz = 4 substeps/frame. Capture = `rep.orchestrator.step(delta_time=0.0, rt_subframes=1, pause_timeline=False)`; advance = `simulation_app.update()`.
- **Conveyor belt** — kinematic rigid body driven by `isaacsim.asset.gen.conveyor.create_conveyor_belt(stage, prim)`. Speed via OG graph variable `graph:variable:Velocity`; direction via node `inputs:direction`. NOT the custom Warp sample in `standalone_examples/conveyor_belt/` (project does not use that — see ADR-0001).
- **Settled** — termination condition: positional drift over a sliding window < `STABILITY_EPSILON` (`common/stability.py:is_settled`).
- **Stereo rig** — two cameras (left/right) sharing a `look_at`, offset ±`STEREO_BASELINE/2` along the rig's horizontal right axis. Toed-in (shared look_at) by default; parallel-shift is a future option (ADR-0002). One `BasicWriter` writes both eyes into `<scenario>/Left/rgb/` and `<scenario>/Right/rgb/` subdirs. Enables stereo/disparity downstream without re-running sims.

## Invariants (every scenario script)
- Headless; PathTracing; SPP + OptiX denoiser configured before the render product is created.
- Parametrized via top-of-file constants (no inline magic numbers).
- Outputs PNG frames + MP4 to `_output/<scenario>/` (gitignored). Per-eye layout: `Left/rgb/rgb_%04d.png`, `Right/rgb/rgb_%04d.png`, plus `left.mp4`, `right.mp4`, `stereo_sbs.mp4`.
- Shared logic in `common/` (app, world, asset, replicator, geometry, stability, loop, video).

## Non-goals
- Not a dataset pipeline (RGB only, no labels/annotations).
- Not real-time/interactive (headless batch sim only).
- No tests framework / CI / package manifest — stdlib `unittest` only for pure logic (ADR-0003); scenario scripts are the integration verification.
