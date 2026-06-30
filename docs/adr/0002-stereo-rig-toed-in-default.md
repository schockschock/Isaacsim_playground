# ADR-0002: Toed-in stereo rig as the default

## Status
Accepted — 2026-06-29

## Context
Every scenario script needs two cameras so stereo is available later. There
are two geometrically distinct ways to set up a stereo pair:

1. **Toed-in (converging)** — both cameras share the same `look_at` target.
   Simple with `rep.functional.create.camera(position=..., look_at=...)`.
   Introduces mild keystone distortion and vertical disparity at the edges
   because each camera's image plane is rotated relative to the other.
2. **Parallel-shift** — both cameras share the same forward direction; the
   sensors are offset horizontally. Geometrically cleaner (no keystone), but
   `rep.functional.create.camera` has no direct projection-offset parameter,
   so it requires either manipulating the camera transform + projection after
   creation or using two distinct look_at points along the forward axis.

This is a learning playground, not a production stereo pipeline.

## Decision
Use **toed-in stereo** as the default: both cameras `look_at` the same
target point. `STEREO_BASELINE` (default 6 cm, potato-scale) is a tunable
constant per scenario.

## Consequences
- `common.replicator.setup_stereo_rig` computes left/right eye positions via
  `common.geometry.compute_eye_positions` (pure, unit-tested) and creates two
  cameras with the shared `look_at`. One `BasicWriter.attach([left_rp, right_rp])`
  writes per-eye subdirs `Left/rgb/` and `Right/rgb/` under each scenario's
  `_output/<scenario>/`.
- Keystone distortion is acceptable for the playground's render-and-inspect
  goals. If we later need disparity-correct stereo, switch to parallel-shift
  and document it in a new ADR; the capture loop and writer plumbing are
  camera-count-agnostic and need no change.
- Setting `STEREO_BASELINE = 0` and dropping one render product collapses
  the rig to a single camera with no other code changes.
- Perf cost: PathTracing renders each frame twice (one per RP). Tunable via
  SPP and baseline; the loop is camera-count-agnostic.
