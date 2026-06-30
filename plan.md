# Plan — IsaacSim Playground: compliant + stereo simulation scripts

## Goal
Four canonical headless PathTraced simulations of the potato (freefall, colliders, object_throw, conveyor), each parametrized and exporting PNG + MP4, sharing logic via `common/`, and each rigged with two cameras (left/right, horizontal baseline) so stereo is available if needed later.

---

## Verified facts that shape the plan

- **Multi-camera output is built in.** `BasicWriter.attach([rp1, rp2])` + default `use_common_output_dir=False` writes per-render-product subdirs named after the RP. One writer, two eyes, zero extra plumbing. (Source: `extscache/omni.replicator.core-.../scripts/writers_default/basicwriter.py:389-401`.)
- **Multi-camera pattern is standard** in the standalone examples (`scene_based_sdg.py:204-214`, `object_based_sdg.py:298-321`): create N cameras → `rep.create.render_product(cam, res, name=...)` per camera → `writer.attach([rps])`.
- **No "stereo" keyword** in replicator examples → stereo here = two regular cameras with a horizontal baseline (toed-in via shared `look_at`).
- **`common/asset.py` default potato path 404s** (`/data1/adrien/...`); the live asset is `/data2/adrien/clean_dataset/2R1-1/2R1-1_centered.usd`.
- **MP4 tooling exists but isn't wired**: `ffmpeg 4.4.2` at `/usr/bin/ffmpeg` (system-wide). `imageio 2.37.2` is available **only inside Isaac Sim's python** — system python (miniconda base) has **no** numpy/PIL/imageio/pytest. → `common/video.py` will shell out to `ffmpeg`/`ffprobe` via `subprocess`, so it works in any python and is unit-testable with stdlib only.
- **Conveyor**: README's extension `isaacsim.asset.gen.conveyor` has a clean API — `create_conveyor_belt(stage, prim)` returns an OG node; speed via `graph:variable:Velocity`, direction via `inputs:direction`, kinematic rigid body. The `standalone_examples/conveyor_belt/` Warp sample (9 files, CUDA graphs) is **not** what we use. The extension auto-applies RigidBodyAPI/CollisionAPI/PhysxSurfaceVelocityAPI/MeshCollisionAPI(convexHull), so we must not double-apply.
- **Autofs gotcha**: `/home/adrien/isaacsim` is an autofs mount — `ls /home/adrien/` doesn't show it, but direct access works.
- **PathTracing captures each RP independently**: 2 cameras = 2× per-frame render cost. Tunable via SPP and baseline constants.

---

## TDD strategy

**Principle:** design pure logic test-first; treat Isaac-Sim-bound code as integration validated by running the actual scripts.

**Constraint:** `AGENTS.md` says "No tests, no CI, no package manifests." Resolution — **lightest possible scaffolding**: a `tests/` directory with stdlib `unittest` test files. No pytest, no conftest, no manifest. Run with `python3 -m unittest discover -s tests -v` (system python, stdlib only). This honors "no framework" while still letting us drive pure-logic design with failing tests first.

**Split rule:** any logic worth unit-testing goes in a **dependency-free submodule** (`common/<name>.py` with no `isaacsim`/`omni`/`pxr`/`carb`/`numpy` imports — plain tuples and `math` only). The Isaac-Sim-bound wrapper imports the submodule and calls it. Tests import only the submodule, so they run under system python without launching SimulationApp.

**Testability classification:**

| Code | Testable? | Where |
|---|---|---|
| Stereo rig geometry (eye positions, right-vector) | Yes — pure vector math | `common/geometry.py` + `tests/test_geometry.py` |
| Settled-detection (positional drift window) | Yes — pure comparison | `common/stability.py` + `tests/test_stability.py` |
| `pngs_to_mp4` (ffmpeg shell-out) | Yes — file I/O + ffprobe verify | `common/video.py` + `tests/test_video.py` |
| `setup_stereo_rig` (camera/RP/writer creation) | Integration only | `common/replicator.py` — validated by running scenarios |
| Capture loop (orchestrator.step / app.update) | Integration only | `common/loop.py` — validated by running scenarios |
| Asset load, colliders, physics material | Integration only | `common/asset.py` — validated by running scenarios |
| Conveyor node wiring | Integration only | `samples/simulations/conveyor.py` — validated by running |

**Red-green cycle per pure module:** (1) write `tests/test_<m>.py` with cases, (2) run → fail (module missing), (3) create stub → fail (wrong behavior), (4) implement → pass, (5) refactor. Integration code is written after its pure dependency passes, and verified by Phase 5.1–5.2.

---

## Execution plan

### Phase 0 — `common/` fixes + shared gaps (test-first on pure logic)

| Step | File | What | Test? |
|---|---|---|---|
| 0.1 | `tests/test_geometry.py` (new) | Cases for `rig_right_vector`, `compute_eye_positions`: nominal (view along -Y, right=+X), edge (looking straight down → degenerate cross with up → fallback right axis), zero-baseline (both eyes = base), sign/orientation parity. | write first |
| 0.2 | `common/geometry.py` (new) | Pure-python 3-vector helpers + `rig_right_vector(base, look_at, up=(0,0,1))` and `compute_eye_positions(base, look_at, baseline, up)`. No numpy/Isaac imports. | make 0.1 pass |
| 0.3 | `tests/test_stability.py` (new) | Cases for `is_settled(window, epsilon, required_len)`: identical window → True; one outlier → False; window shorter than `required_len` → False; exactly at threshold → True; empty window → False. | write first |
| 0.4 | `common/stability.py` (new) | Pure-python `is_settled(...)` over a list of (x,y,z) tuples. | make 0.3 pass |
| 0.5 | `tests/test_video.py` (new) | Cases for `pngs_to_mp4`: generates 5 dummy PNGs via `ffmpeg -f lavfi -i testsrc` into a tempdir, calls `pngs_to_mp4`, asserts mp4 exists + `ffprobe` reports 5 frames. Also `stereo_pair_to_mp4` SBS dimensions = 2× width. Uses `unittest.tmpdir` + `subprocess`. | write first |
| 0.6 | `common/video.py` (new) | `pngs_to_mp4(frame_dir, out_path, fps=120, pattern="rgb_%04d.png")` and `stereo_pair_to_mp4(left_dir, right_dir, out_path, fps, mode="sbs")` — both shell out to `ffmpeg` via `subprocess.run`, verify with `ffprobe`. No imageio. | make 0.5 pass |
| 0.7 | `common/asset.py` | Fix default `prim_path` → `/data2/adrien/clean_dataset/2R1-1/2R1-1_centered.usd` (keep overridable). | verify with `ls` only |
| 0.8 | `common/replicator.py` | Replace single-camera `setup_replicator` with `setup_stereo_rig(...)` — imports `common.geometry` for eye positions, then does the Isaac-Sim-bound camera/RP/writer creation. | integration |
| 0.9 | `common/loop.py` (new) | Shared capture loop — imports `common.stability` for the settled check; the rest (orchestrator.step, app.update, readback) is Isaac-Sim-bound. | integration |
| 0.10 | Module constant | `STEREO_BASELINE = 0.06` (6 cm, potato-scale; tunable per scenario script). | — |

**`setup_stereo_rig` interface:**
```python
def setup_stereo_rig(base_position, look_at, resolution,
                     baseline=STEREO_BASELINE, focal_length=24.0,
                     output_subdir="replicator_output"):
```
- Call `common.geometry.compute_eye_positions(...)` for left/right eye positions (pure, tested).
- Create two cameras (`Left`, `Right`) under `/World/Cameras`, both `look_at` the same target (toed-in stereo).
- `rep.create.render_product(cam, res, name="Left"|"Right")` → list. **Renderer config must be applied BEFORE this call** (existing gotcha — set once before creating either RP).
- `writer.attach([left_rp, right_rp])` → per-eye subdirs `Left/`, `Right/` under `_output/<scenario>/`.
- Return `(left_cam, right_cam, [left_rp, right_rp], writer)`.

**Shared capture loop (`common/loop.py`):**
```
play timeline → warmup (5 frames) → for each frame:
  rep.orchestrator.step(delta_time=0.0, rt_subframes=1, pause_timeline=False)
  readback + common.stability.is_settled(...) check
  simulation_app.update()
  if settled or max_frames → break
→ wait_until_complete → detach → destroy rps
```
The loop is camera-count-agnostic — both eyes are rendered per `step` automatically.

**Phase 0 exit criterion:** `python3 -m unittest discover -s tests -v` is green; `common/asset.py` default path exists; `common/replicator.py` and `common/loop.py` are importable under Isaac Sim's python (smoke check: `$ISAAC_SIM/python.sh -c "from common.replicator import setup_stereo_rig; from common.loop import run_capture_loop"` — does not launch the app).

---

### Phase 1 — freefall (compliant copy)

**File:** `samples/simulations/freefall.py`

Legacy `samples/freefall.py` untouched.

- **Drop height:** ~0.4 m
- **Initial spin:** ~5 rad/s about Z
- **Material:** restitution 0.35, friction 0.6
- **Stereo rig:** base position above and to the side of the impact zone, looking at the landing point. Baseline 6 cm.
- **Capture:** shared loop from `common/loop.py`.
- **Output:** `_output/freefall/Left/rgb_*.png`, `_output/freefall/Right/rgb_*.png`, `_output/freefall/left.mp4`, `_output/freefall/right.mp4`.
- **Termination:** settled detection (STABILITY_EPSILON) or MAX_FRAMES.
- **TDD note:** no new unit tests here — the testable math already passed in Phase 0. This script is integration; validated by Phase 5.1.

---

### Phase 2 — colliders (3 approximations, side view)

**File:** `samples/simulations/colliders.py`

- Three potato prims side-by-side at x ≈ {-0.15, 0, +0.15}.
- Collider approximations: `[convexHull, convexDecomposition, sdf]`.
- Identical drop height + material (same as freefall).
- **Stereo rig:** side view, base position along the ±Y axis from the row midpoint, looking at the row center. Baseline horizontal along X (the row axis) so both eyes see all three prims with parallax.
- Otherwise reuses freefall's flow and capture loop.

---

### Phase 3 — object_throw (tunable launch + wall rebound)

**File:** `samples/simulations/object_throw.py`

- Tunable `INIT_LINEAR_VELOCITY` + `INIT_ANGULAR_VELOCITY` as top-of-file constants.
- A vertical wall prim (box with collider, fixed/kinematic) placed across the path: potato rebounds off the wall then off the ground.
- **Stereo rig:** wide framing covering wall + floor impact zones; baseline horizontal perpendicular to trajectory plane.
- Wall position/height, initial velocity, material, and camera constants all at top of file.

---

### Phase 4 — conveyor (extension, not the Warp sample)

**File:** `samples/simulations/conveyor.py`

- Build a long box prim as the conveyor belt; make it a kinematic rigid body.
- `create_conveyor_belt(stage, belt_prim)` → OG node. **Do not re-apply** RigidBody/Collision/SurfaceVelocity/MeshCollision APIs — the extension handles them.
- Set speed via `stage.GetPrimAtPath(".../ConveyorBeltGraph").GetAttribute("graph:variable:Velocity").Set(v)` and direction via `conveyor_node.GetAttribute("inputs:direction").Set(...)`.
- Place the potato on the belt surface (above belt center). Belt transports it via surface velocity; potato rolls then falls off the end into a drop zone.
- **Stereo rig:** framing belt + drop zone; baseline horizontal across the belt width.
- Capture loop + MP4, same as other scenarios.

---

### Phase 5 — verify + reconcile docs

| Step | What |
|---|---|
| 5.0 | `python3 -m unittest discover -s tests -v` — pure-logic suite still green. |
| 5.1 | Run each: `conda deactivate && $ISAAC_SIM/python.sh samples/simulations/<script>.py` |
| 5.2 | Confirm `_output/<scenario>/{Left,Right}/rgb_*.png` + `_output/<scenario>/{left,right}.mp4` exist and look correct (frame counts match via `ffprobe`). |
| 5.3 | Write `docs/adr/0001-conveyor-extension-over-warp-sample.md` |
| 5.4 | Write `docs/adr/0002-stereo-rig-toed-in-default.md` |
| 5.5 | Write `docs/adr/0003-stdlib-unittest-over-pytest.md` (why we chose stdlib `unittest` + ffmpeg subprocess over pytest/imageio: honors AGENTS.md "no manifest", system python has no numpy/imageio, ffmpeg is already on PATH). |
| 5.6 | Update `AGENTS.md`: autofs mount one-liner; conveyor-extension note (not the Warp sample); per-eye output subdir convention; one line on the `tests/` suite (`python3 -m unittest discover -s tests`) and the pure-logic split rule. |
| 5.7 | Update `README.md`: point at compliant `samples/simulations/*.py`; flag legacy `samples/freefall.py` as non-compliant. |

(No lint/typecheck pass — the repo has no lint/typecheck configuration per `AGENTS.md`. The `unittest` run **is** the verification step for pure logic; running each scenario script is the integration verification.)

---

## File layout after execution

```
CONTEXT.md                                     (domain glossary — already approved)
plan.md                                        (this file)

samples/
  freefall.py                                  (legacy, untouched)

  simulations/
    freefall.py                                (compliant, stereo)
    colliders.py                               (stereo)
    object_throw.py                            (stereo)
    conveyor.py                                (stereo)

common/
  __init__.py
  app.py                                       (unchanged)
  world.py                                     (unchanged)
  asset.py                                     (fix default prim_path)
  replicator.py                                (add setup_stereo_rig; imports common.geometry)
  loop.py                                      (new — shared capture loop; imports common.stability)
  geometry.py                                  (new, pure — rig_right_vector, compute_eye_positions)
  stability.py                                 (new, pure — is_settled)
  video.py                                     (new — pngs_to_mp4 + stereo_pair_to_mp4 via ffmpeg subprocess)

tests/                                         (new — stdlib unittest, runs under system python)
  __init__.py
  test_geometry.py                             (eye positions, right-vector, edge cases)
  test_stability.py                            (settled window detection)
  test_video.py                                (ffmpeg-driven pngs→mp4 round-trip + ffprobe verify)

_output/                                       (gitignored)
  freefall/
    Left/rgb_*.png
    Right/rgb_*.png
    left.mp4
    right.mp4
  colliders/  (same structure)
  object_throw/ (same structure)
  conveyor/   (same structure)

docs/
  agents/ (unchanged)
  adr/
    0001-conveyor-extension-over-warp-sample.md
    0002-stereo-rig-toed-in-default.md
    0003-stdlib-unittest-over-pytest.md
```

## Stereo design choice: toed-in (confirmed)

Both cameras `look_at` the same target point — the simplest approach with `rep.create.camera`'s API. This introduces mild keystone/vertical-disparity distortion at the edges, acceptable for a learning playground. `STEREO_BASELINE` is tunable per scenario (constant at top of each script). Parallel-shift stereo (same forward direction, sensor shift) is a future option, documented in ADR-0002.

**Perf:** 2 cameras = PathTracing renders each frame twice. If render time becomes a problem, the rig is one constant away from single-camera (set `baseline=0`, drop one RP) — the capture loop is camera-count-agnostic.

## TDD design choice: stdlib unittest + ffmpeg subprocess (confirmed)

- **No pytest, no imageio, no numpy** in system python → stdlib `unittest` + `subprocess`-to-`ffmpeg` only.
- Pure logic lives in dependency-free submodules (`common/geometry.py`, `common/stability.py`) so tests can import them without launching SimulationApp.
- Isaac-Sim-bound code (camera/RP/writer/loop/asset) is integration-tested by running the scenario scripts (Phase 5.1).
- `common/video.py` shells out to `ffmpeg`/`ffprobe` instead of using `imageio` — works in any python, unit-testable with stdlib, no dependency on Isaac Sim's bundled python.
- Rationale recorded in ADR-0003.
