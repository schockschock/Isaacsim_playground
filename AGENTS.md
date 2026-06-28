# AGENTS.md

## What this is
Personal learning playground for NVIDIA Isaac Sim 6.0.1, PhysX, and Omniverse Kit APIs.
No tests, no CI, no package manifests — scripts are one-off against the local Isaac Sim install.
Planned objectives (see README.md): (1) load scene + USD from server + replicator pictures;
(2) compare collider types (MeshSimplification, ConvexHull, ConvexDecomposition, SDF) on falling objects;
(3) conveyor belt spawning objects + replicator.

## Environment (required, non-obvious)
- Isaac Sim lives at `/home/adrien/isaacsim` (full build in `$ISAAC_SIM/VERSION`). Get the path with `source .env`.
- Run scripts with Isaac Sim's bundled Python, NEVER system/conda python:
  `$ISAAC_SIM/python.sh my_script.py`
- `python.sh` warns/misbehaves if `CONDA_PREFIX` is set → `conda deactivate` first.
- No display on this server → every script must launch headless:
  `SimulationApp({"headless": True})`
- Instantiate `SimulationApp` BEFORE importing any other `isaacsim.*` module (runtime imports come after app creation).

## Reference examples (local — copy these patterns)
- Replicator / synthetic data: `$ISAAC_SIM/standalone_examples/replicator/`
- Conveyor belt: `$ISAAC_SIM/standalone_examples/conveyor_belt/` (maps to objective #3)
- Getting-started tutorials: `$ISAAC_SIM/standalone_examples/tutorials/getting_started/`

## Repo gotchas
- `.env` is tracked in git and holds a machine-absolute path — don't put secrets here.
- `.gitignore` is empty; add patterns as artifacts appear (`__pycache__/`, `*.usd`, `_output/`, etc.).

## Agent skills

### Issue tracker

Issues live in GitHub Issues (repo: schockschock/Isaacsim_playground). PRs are not a triage surface. See `docs/agents/issue-tracker.md`.

### Triage labels

Default vocabulary: needs-triage, needs-info, ready-for-agent, ready-for-human, wontfix. See `docs/agents/triage-labels.md`.

### Domain docs

Single-context — one CONTEXT.md + docs/adr/ at the repo root. See `docs/agents/domain.md`.
