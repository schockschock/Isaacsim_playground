# AGENTS.md

## What this is
Personal learning playground for NVIDIA Isaac Sim 6.0.1 (see `/home/adrien/isaacsim/VERSION`), PhysX, and Omniverse Kit APIs.
No CI, no package manifests — scripts run one-off against the local Isaac Sim install. Pure-logic unit tests exist (`tests/`, stdlib `unittest`); see "Verification" below.
Target scripts (see README.md, CONTEXT.md): freefall, colliders, object_throw, conveyor — all under `samples/simulations/`.

## Environment (required, non-obvious)
- Isaac Sim lives at `/home/adrien/isaacsim`; `. .env` exports `ISAAC_SIM`. **Autofs mount**: `ls /home/adrien/` won't show it, but direct access (`ls $ISAAC_SIM`, `cat $ISAAC_SIM/VERSION`) works.
- Run scripts ONLY with Isaac Sim's bundled Python, never system/conda python:
  `$ISAAC_SIM/python.sh my_script.py`
- `python.sh` misbehaves if `CONDA_PREFIX` is set → `conda deactivate` first. (In `sh`, use `. .env` not `source .env`.)
- No display on this server → every script must launch `SimulationApp({"headless": True})`
  BEFORE importing any `isaacsim.*` / `omni.*` / `pxr.*` module (runtime imports come after app creation).

## Code layout
- `common/` is the reusable layer — import from it, don't duplicate:
  - `app.py` — `launch()`/`shutdown()` SimulationApp lifecycle (headless).
  - `world.py` — stage + physics scene + renderer config (`setup_stage`, `configure_renderer`).
  - `asset.py` — potato USD load, colliders, rigid body, physics material, state readback.
  - `replicator.py` — camera, BasicWriter, render product, `setup_stereo_rig` (toed-in two-camera rig).
  - `geometry.py` — pure stereo rig vector math (no deps; unit-tested).
  - `stability.py` — pure settled-detection (no deps; unit-tested).
  - `loop.py` — shared capture loop (orchestrator.step + app.update + settled check).
  - `video.py` — `pngs_to_mp4` / `stereo_pair_to_mp4` via ffmpeg subprocess (no deps; unit-tested).
- `samples/simulations/` holds the compliant scenario scripts (freefall, colliders, object_throw, conveyor).
  `samples/freefall.py` is a LEGACY pre-`common/` version: `headless: False`, hardcoded output — do NOT copy as a template.
  `get_started.py`, `sdg_example.py`, `PhysicEngine.py` are tutorial-style explorations (also `headless: False`).
- `tests/` — stdlib `unittest` for pure logic only (`common/geometry.py`, `common/stability.py`, `common/video.py`).

## Script requirements (from README.md — mandatory for every sim script)
- Headless; render mode = `PathTracing` (RealTime's TAA ghosts fast-moving objects).
- Parametrize via constants/config at top of file, not inline magic numbers.
- Save PNG frames + MP4 of the simulation (per eye: `Left/rgb/`, `Right/rgb/` + `left.mp4`, `right.mp4`, `stereo_sbs.mp4`).
- Shared logic goes in `common/`, not per-script.

## Hard-won gotchas
- Apply renderer settings BEFORE `rep.create.render_product()` — the render product snapshots
  current config. See `common/world.py:configure_renderer`.
- PhysX rates: timeline 120 fps, internal solver 480 Hz → 4 substeps/frame. Capture with
  `rep.orchestrator.step(delta_time=0.0, rt_subframes=1, pause_timeline=False)`, then
  `simulation_app.update()` to advance one frame.
- Colliders need 3 APIs per mesh: `UsdPhysics.CollisionAPI` + `PhysxSchema.PhysxCollisionAPI`
  + `UsdPhysics.MeshCollisionAPI` (approximation = `convexHull` | `convexDecomposition` | `sdf`).
  See `common/asset.py:add_colliders`.
- Physics materials bind via the `material:binding:physics` relationship, NOT `material:binding`
  (that's for rendering shaders). See `common/asset.py:apply_physics_material`.
- `common/asset.py` default potato path is machine-absolute (`/data2/adrien/...`) — override `prim_path` or it 404s on other machines.
- `BasicWriter` does NOT accept `depth`/`flow` kwargs; use `distance_to_image_plane`, `instance_segmentation`, etc. (see `common/replicator.py:setup_writer`). With multiple render products it writes one subdir per RP (`<RP_name>/rgb/rgb_%04d.png`).
- Conveyor: use the `isaacsim.asset.gen.conveyor` extension (`create_conveyor_belt`), NOT the `standalone_examples/conveyor_belt/` Warp sample (ADR-0001). The extension is not enabled by default — load via `omni.kit.app.get_extension_manager().set_extension_enabled_immediate(...)` BEFORE importing it (defer the import). `create_conveyor_belt` only applies CollisionAPI+PhysxSurfaceVelocityAPI if the prim LACKS RigidBodyAPI — set kinematic AFTER calling it, not before, or the belt silently gets no collider.
- `UsdGeom.Cube` final extents = `size * scale`; use size=1.0 and scale by full dimensions, or the geometry is far smaller than intended.

## Verification
- Pure logic: `python3 -m unittest discover -s tests -v` (system python, ~1s, no GPU). 22 tests.
- Integration: `conda deactivate && . .env && $ISAAC_SIM/python.sh samples/simulations/<script>.py` — confirm `_output/<scenario>/{Left,Right}/rgb/rgb_*.png` + the three MP4s.

## Reference examples (local — copy these patterns)
- `$ISAAC_SIM/standalone_examples/replicator/` — synthetic data / writers (multi-camera: `scene_based_sdg.py`, `object_based_sdg.py`).
- `$ISAAC_SIM/standalone_examples/conveyor_belt/` — Warp-kernel conveyor sample (NOT used here; see ADR-0001 for why we use the extension instead).
- `$ISAAC_SIM/standalone_examples/tutorials/getting_started/` — hello-world flows.

## Repo gotchas
- `.env` is tracked in git and holds a machine-absolute `ISAAC_SIM` path — don't put secrets here.
- `.gitignore` covers `_output/`, `__pycache__/`, `*.pyc`, `*.png`, `*.csv`, `*.mp4` (artifacts stay local).

## Agent skills
- Issue tracker: GitHub Issues (repo: schockschock/Isaacsim_playground); PRs are NOT a triage surface. See `docs/agents/issue-tracker.md`.
- Triage labels: needs-triage, needs-info, ready-for-agent, ready-for-human, wontfix. See `docs/agents/triage-labels.md`.
- Domain docs: single-context — `CONTEXT.md` + `docs/adr/` at repo root. See `docs/agents/domain.md`.
