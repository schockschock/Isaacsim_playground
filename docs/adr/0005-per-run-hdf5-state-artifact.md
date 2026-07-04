# ADR-0005: Per-run HDF5 state artifact

## Status
Accepted — 2026-07-04

## Context
Each simulation run produces RGB frames + MP4 videos, but the PhysX ground-truth
state (position, orientation, linear/angular velocity, energy, contacts, etc.)
has no persistent artifact. Downstream validation, training data pipelines, and
multi-run comparisons require a machine-readable, self-describing container for
the full per-frame state.

## Decision
Write one HDF5 `.h5` per run to `<output_dir>/<scenario>.h5` alongside the
existing PNG/MP4 artifacts. The container uses a structured numpy dtype for
compact row access and top-level attributes for run metadata.

### Schema

**Top-level attributes:**

| Key | Type | Description |
|-----|------|-------------|
| `scenario` | str | Canonical scenario name (e.g. "freefall") |
| `fps` | int | Capture frame rate (120) |
| `dt_phys` | float | PhysX internal timestep (1/480) |
| `solver_substeps` | int | Substeps per capture frame (4) |
| `t_offset_frames` | int | State rows logged before first RGB frame |
| `n_state_rows` | int | Total rows written (written at finalize) |
| `n_rgb_frames` | int | RGB frames captured (written at finalize) |
| `baseline_m` | float | Stereo baseline in meters |
| `material_restitution` | float | Potato restitution coefficient |
| `material_friction` | float | Potato friction coefficient |
| `material_density` | float | Potato density (kg/m³) |
| `potato_usd_path` | str | Path to the potato USD asset |

**`/state` dataset** — structured array, shape `(T,)`, dtype:

| Field | Shape | Unit | Description |
|-------|-------|------|-------------|
| `time` | scalar | s | Simulation time |
| `position` | (3,) | m | World COM position |
| `orientation` | (4,) | – | World quaternion (wxyz) |
| `lin_velocity` | (3,) | m/s | World linear velocity at COM |
| `ang_velocity` | (3,) | rad/s | World angular velocity |

### Writing semantics
- State is logged from sim t=0 (during warmup frames) so every physics step is
  recorded. Warmup state rows have no corresponding RGB frame; the offset is
  recorded in `t_offset_frames`.
- `DatasetWriter.finalize()` flushes buffered rows and writes `n_state_rows` and
  `n_rgb_frames` as attributes.

## Consequences
- `h5py` becomes a required dependency (already ships with Isaac Sim's python).
- `DatasetWriter` is pure h5py+numpy — no Isaac deps, testable with stdlib unittest.
- Schema is designed for incremental deepening: `/state` expands in Slice B
  (mass, inertia, energy) and Slice C (in_contact). Older readers ignore
  unknown fields gracefully.
