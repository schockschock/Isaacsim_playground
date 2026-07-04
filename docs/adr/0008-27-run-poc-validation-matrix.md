# ADR-0008: 27-run validation matrix with argparse driver

## Status
Accepted — 2026-07-04

## Context

Slice A through F produced per-run HDF5 artifacts with full state, contacts,
energy, and modalities — but only for single initial conditions. To validate
the pipeline across a meaningful IC space, we need to test 3 velocity levels
× 3 initial poses × 3 scenarios = 27 independent runs.

A single-process loop over all 27 ICs inside one `SimulationApp` invocation
is risky: `SimulationApp` must be created before any Omniverse import, and
re-entering the app lifecycle per IC is untested. An outer driver script that
launches each run as a separate process is safer and mirrors how Isaac Sim
pipelines are typically orchestrated.

## Decision

Use an **outer driver** pattern with argparse `--run V P` switches.

- Each scenario script (`freefall.py`, `conveyor.py`, `object_throw.py`)
  accepts `--run <velocity_level> <pose_index>`.
- The driver (`tests/integration/validate_all.py`) launches each run via
  `$ISAAC_SIM/python.sh <script> --run V P`, then validates the resulting
  HDF5 via the per-scenario `validate_<scenario>.py` script.
- Output goes to `_output/<scenario>/v{V}_p{P}/<scenario>_v{V}_p{P}.h5`.

### IC grid (locked)

**Poses** (same across all 3 scenarios):

| Index | Quaternion (wxyz) | Description |
|-------|-------------------|-------------|
| P0 | `(1, 0, 0, 0)` | Rest orientation |
| P1 | `(0.924, 0.383, 0, 0)` | 45° tilt about X — asymmetric contact |
| P2 | `(0.5, 0.5, 0.5, 0.5)` | 120° about (1,1,1) — threefold symmetry breaking |

**Velocities** (scenario-tuned):

| Scenario | V0 | V1 | V2 | Axis |
|----------|----|----|----|------|
| freefall | ω=0 | ω=10 | ω=20 | (0.259, 0.432, 0.864) |
| conveyor | v_belt=0.2 | v_belt=0.5 | v_belt=1.0 | belt X direction |
| object_throw | v_lin=2 | v_lin=5 | v_lin=9 | (0.995, 0, 0.0995) |

## Consequences

- The driver is the single entry point for cluster/batch runs.
- Individual runs remain independently-launchable for debugging.
- 27 full sims take ~2-5 hours with all modalities enabled (4-12 min each).
- Default (non-`--run`) scripts continue to work unchanged for interactive
  development.
- The argparse `--run` switch is parsed via `parse_known_args()` so Isaac
  Sim's own kit arguments are forwarded correctly.
