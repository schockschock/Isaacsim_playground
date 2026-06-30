# IsaacSim Playground

Headless physics simulations and synthetic-data capture for building datasets — Isaac Sim 6.0.1 + PhysX + Omniverse Kit.

## Requirements

- Isaac Sim 6.0.1: https://docs.isaacsim.omniverse.nvidia.com/latest/index.html
- `.env` at the repo root exporting `ISAAC_SIM` (machine-absolute path). The install lives at `/home/adrien/isaacsim` (an autofs mount — `ls /home/adrien` won't show it; direct access works).

## Quick start

```sh
conda deactivate
. .env
$ISAAC_SIM/python.sh ./simulation_scripts/<script>.py
```

Each script writes per-eye PNG frames + MP4 videos to `_output/<scenario>/`:

```
_output/<scenario>/
├── Left/rgb/rgb_%04d.png
├── Right/rgb/rgb_%04d.png
├── left.mp4
├── right.mp4
└── stereo_sbs.mp4
```

## Scripts

| Script | What it does |
|---|---|
| `simulation_scripts/freefall.py` | Potato dropped onto the gridroom ground with initial spin. Camera above the impact zone. |
| `simulation_scripts/colliders.py` | Three potatoes dropped side-by-side, one per collider approximation (`convexHull`, `convexDecomposition`, `sdf`). Side-view camera. |
| `simulation_scripts/object_throw.py` | Potato launched sideways with tunable linear + angular velocity, rebounding off a vertical wall then the ground. Wide camera. |
| `simulation_scripts/conveyor.py` | Potato transported by a conveyor belt (isaacsim.asset.gen.conveyor extension), falling off the end. Camera frames belt + drop zone. |

All scripts use a **two-camera stereo rig** (toed-in, default 6 cm baseline) so left/right data is available downstream.

## Mandatory rules (every script in this repo)

- Headless (`SimulationApp({"headless": True})` before any isaacsim/omni/pxr import).
- Render mode = `PathTracing` (not RealTime — TAA ghosts fast-moving objects).
- Parametrized via constants at the top-of-file (no inline magic numbers).
- Outputs PNG frames + MP4 for both eyes.
- Shared logic goes in `common/`, never duplicated per-script.

## Repo layout

| Directory / file | Purpose |
|---|---|
| `simulation_scripts/` | The four canonical dataset-generating scripts. |
| `common/` | Shared layer — app lifecycle, stage setup, asset loading, replicator, stereo rig geometry, stability detection, capture loop, MP4 encoder. |
| `tests/` | Stdlib `unittest` for pure logic (`common/geometry`, `common/stability`, `common/video`). Run with `python3 -m unittest discover -s tests -v` (system python, ~1s, no GPU). |
| `docs/adr/` | Architecture Decision Records (why we chose the conveyor extension over the Warp sample, toed-in stereo, stdlib unittest). |
| `CONTEXT.md` | Domain glossary — terms, canonical scenarios, invariants. |
| `AGENTS.md` | Agent-facing reference — environment, gotchas, commands, code layout. |
| `samples/` | Pre-`common/` exploratory scripts (`headless: False`, do not copy as templates). |
