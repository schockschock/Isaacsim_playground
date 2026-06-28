# Plan — Isaacsim_playground

Three learning objectives for NVIDIA Isaac Sim 6.0.1, PhysX, and Omniverse Kit APIs.
Built sequentially (1 → 2 → 3); each reuses the shared `common/` package and the previous layer.

## Confirmed decisions (from grilling)

| Dimension | Decision |
|---|---|
| Asset | `/data1/adrien/clean_dataset/2R1-1/2R1-1_centered.usd` (3.5 MB, textures in `textures/`, centered origin) |
| Asset loading | Load **in place** — textures resolve via relative paths; don't copy the USD alone |
| Scene | Empty stage + ground plane + **dome light (shadows off)** + physics world |
| Lighting | Single dome light, intensity 500, `inputs:shadow:enabled = false` (pattern from SDG Workflows doc) |
| Scale | Real-world; cm-scale object; drop 1m; 9.8 m/s² |
| Build order | #1 → #2 → #3, each reusing the previous layer |
| #2 done-criteria | Side-by-side photos (every Nth physics step, in-phase) **+** logged per-step metrics (bounce height, contact timestamps, linear velocity) |
| Capture sync | Physics-step callback; capture every Nth step (submultiple) so it's slower but in phase |
| Output | `_simulatoin_results/` in repo (git-ignored) |
| Runtime | `$ISAAC_SIM/python.sh`, `conda deactivate` first, `SimulationApp({"headless": True})` before any `isaacsim.*` import |

## File layout

```
Isaacsim_playground/
├── _simulatoin_results/                      # git-ignored; photos + metrics land here
├── common/
│   ├── __init__.py
│   ├── app.py                    # headless SimulationApp launcher + teardown
│   ├── world.py                  # ground plane + dome light (no shadows) + physics world
│   ├── asset.py                  # load the potato USD in place, return prim
│   └── replicator.py             # camera + writer helpers → _simulatoin_results/
├── obj1_scene_usd_replicator.py  # objective 1
├── obj2_collider_comparison.py   # objective 2 (the meaty one)
└── obj3_conveyor_belt.py         # objective 3
```

A small `common/` package absorbs the shared boilerplate (AGENTS.md gotchas: SimulationApp-first, headless, world setup). Each objective script stays focused on its own logic and imports from `common/`.

## Objective 1 — `obj1_scene_usd_replicator.py` (foundation)

1. Launch `SimulationApp({"headless": True})` → then import `isaacsim.*`.
2. Build empty world: ground plane (`/World/GroundPlane`), dome light (shadows off), `PhysicsContext` with gravity -9.8, fixed timestep.
3. Load `/data1/adrien/clean_dataset/2R1-1/2R1-1_centered.usd` via `add_reference_to_stage` (in place, so `textures/` resolves).
4. Place the potato at a visible spot.
5. Replicator: one camera looking at the object, `rep.Writer` writing PNGs to `_simulatoin_results/obj1/`. Capture ~30 frames while world steps.
6. Clean shutdown.

**Done** = photos of the loaded textured potato in an empty world appear in `_simulatoin_results/obj1/`.

## Objective 2 — `obj2_collider_comparison.py` (reuses #1's world/asset/replicator)

1. Reuse `common.world` for scene + physics.
2. Load the potato USD **4 times** as 4 sibling prims, spaced side-by-side along X.
3. On each, apply a different collision approximation (Isaac Sim 6.0 collision API):
   - A: `MeshSimplification`
   - B: `ConvexHull`
   - C: `ConvexDecomposition`
   - D: `SDF` (signed-distance-field)
   - Exact attribute names confirmed against the Isaac Sim 6.0 collision API during build.
4. Add `RigidBodyAPI` to all 4; place at drop height (~1.2 m), zero initial velocity.
5. Register a `world.add_physics_step_callback` that:
   - **Every step**: append per-object metrics to an in-memory log — world position Y (bounce height), linear velocity, and contact-event timestamps (via contact report API).
   - **Every Nth step** (N = submultiple, e.g. 4): trigger replicator capture so the frame is in phase with physics but at a slower render rate.
6. Run ~3000 physics steps (enough for several bounces to settle).
7. Write photos to `_simulatoin_results/obj2/` and metrics to `_simulatoin_results/obj2/metrics.csv` (columns: step, time, object, pos_y, vel_y, contact_flag).
8. Clean shutdown.

**Done** = `_simulatoin_results/obj2/` contains in-phase side-on photos of the 4 drops **+** a CSV to plot bounce decay per collider.

## Objective 3 — `obj3_conveyor_belt.py` (reuses #1's world/asset/replicator)

1. Reuse `common.world`.
2. Build a conveyor belt using `$ISAAC_SIM/standalone_examples/conveyor_belt/` as the reference pattern (`cb_actuators.py`, `cb_body_manager.py`, `cb_conveyor_belt_manager.py` — copy the belt-construction pattern, not the whole app).
3. Spawn the potato USD onto the belt surface.
4. Drive the belt; replicator captures to `_simulatoin_results/obj3/`.
5. Clean shutdown.

**Done** = `_simulatoin_results/obj3/` shows the potato being carried along the belt.

## Order of work

1. `common/` package + `obj1` end-to-end (proves the headless + asset + replicator pipeline).
   **Gate:** photos appear in `_simulatoin_results/obj1/`.
2. `obj2` on top of `common/` (adds collider API + physics-step-synced capture + metrics).
   **Gate:** CSV + in-phase photos in `_simulatoin_results/obj2/`.
3. `obj3` on top of `common/` (adds conveyor belt).
   **Gate:** photos in `_simulatoin_results/obj3/`.

## Reference examples (local — copy these patterns)

- Replicator / synthetic data: `$ISAAC_SIM/standalone_examples/replicator/`
- Conveyor belt: `$ISAAC_SIM/standalone_examples/conveyor_belt/` (maps to objective #3)
- Getting-started tutorials: `$ISAAC_SIM/standalone_examples/tutorials/getting_started/`

## Reference examples (web)

- SDG Workflows (dome light + capture-on-play + render-product gating):
  https://docs.isaacsim.omniverse.nvidia.com/latest/replicator_tutorials/tutorial_replicator_sdg_workflows.html

## Open items (resolve during build, not blocking)

- Exact Isaac Sim 6.0 collision-approximation attribute names for the 4 collider types.
- Conveyor belt: whether to reuse the standalone example's actuator/body-manager classes verbatim or slim them down.
- Tunables with sensible defaults: drop height, N (capture submultiple), total steps, camera resolution, belt speed.
