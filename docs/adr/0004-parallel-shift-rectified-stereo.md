# ADR-0004: Parallel-shift rectified stereo (supersedes ADR-0002)

## Status
Accepted — 2026-07-04

## Context

ADR-0002 chose a **toed-in** stereo rig for simplicity: both cameras shared
the same `look_at` target, introducing mild keystone distortion and vertical
disparity at the edges.

This works for RGB-only datasets but fails for ground-truth disparity
computation (Slice E). Disparity requires:
- Parallel optical axes (no relative rotation between cameras)
- Zero vertical disparity (epipolar lines are horizontal)
- `d = f_px · B / Z` to hold exactly per pixel

With toed-in cameras, the image planes are rotated relative to each other,
so disparity requires full reprojection to a common rectified frame — which
is doable but adds complexity and numerical error.

## Decision

Switch to a **parallel-shift rectified** stereo rig:

- Both cameras share the same forward direction vector (no convergence rotation).
- Eyes are offset by ±baseline/2 along the rig's right vector.
- Each camera's `look_at` target is placed far ahead along the forward axis
  (1000 m), effectively making the optical axes parallel.

Implementation in `common/replicator.py:setup_stereo_rig`:

```python
forward = normalize(look_at - base_position)
left_look_at  = left_pos  + forward * 1000
right_look_at = right_pos + forward * 1000
```

### Configuration changes
- Default stereo baseline: 0.06 m → **0.08 m** (wider separation for
  stronger disparity signal at potato-scale distances).
- Focal length remains 24 mm. Resolution remains per-scenario configurable
  but defaults to 640×480 where the scenario doesn't override.
- All scenario scripts continue to work with the same `setup_stereo_rig`
  interface (the rig-geometry change is internal).

## Consequences

- **Supersedes** ADR-0002 (toed-in default). ADR-0002 is retained as
  superseded reference, not deleted.
- Disparity computation in Slice E can use the simple `d = f·B/Z` formula
  without rectification preprocessing.
- Scene framing changes slightly: the parallel-shift rig sees a slightly
  wider overlap region with no keystone.
- The `compute_eye_positions` function in `common/geometry.py` is unchanged
  (still computes horizontal offsets along the right vector).
- Camera `look_at` targets are no longer shared between eyes; the writer
  still attaches to both render products.
