# ADR-0009: Measurement-correctness test families

## Status
Accepted — 2026-07-04

## Context

The dataset pipeline records per-frame PhysX state (position, orientation,
velocity, mass, inertia, energy, contacts) into HDF5. Without validation,
silent errors in readback, unit conversion, or physics configuration could
corrupt the entire dataset. We need a suite of physics-informed integration
tests that verify the correctness of recorded measurements.

These tests run **under Isaac Sim** (integration tests, not stdlib unittest)
because they require the PhysX solver output. They are invoked by
`validate_<scenario>.py` scripts that load a per-run HDF5 and assert
physical invariants.

## Decision

Five assertion families, applied to every scenario run:

### 1. Mass/inertia invariant
- **What**: `std(mass) < 1e-9` across all frames.
- **Why**: Rigid-body mass is invariant. Any drift indicates a Tensor API
  or readback bug.

### 2. Energy conservation
- **What**: Between `dissipating == False` frames, `|ΔE_total| < 0.01 J`.
- **Why**: During free-flight (no contacts), total mechanical energy is
  conserved (trade between KE and PE). The dissipating flag (Slice B)
  gates out contact frames automatically.

### 3. Finite-difference consistency
- **What**: `|v_lin - Δpos·fps| < ε_lin` and `|ω_tensor - δq/Δt| < ε_ang`
  for non-contact frames.
- **Why**: The Tensor API reports the solver's internal state. The finite
  difference from consecutive state rows should match within reasonable
  numerical error.
- **Concession**: ε_ang is relaxed for scenarios with extreme angular
  velocities (object_throw, up to 120 rad/s) where the quaternion-based
  finite difference degrades (1 radian/frame at 120 fps).

### 4. Freefall analytic
- **What**: `|z(t) - (z0 - ½gt²)| < ε` for non-contact ballistic frames.
- **Why**: The simplest analytic validation — a falling body under uniform
  gravity follows a parabola. Any deviation indicates a PhysX configuration
  bug (wrong gravity, timestep mismatch).
- **Applicability**: All three scenarios (freefall's fall segment,
  conveyor's airborne fall-off, object_throw's ballistic flight).

### 5. Contact-energy agreement
- **What**: Pearson r between `|ΔKE|` and `Σ|impulse|` across impact frames
  > 0.7 (or 0.6 for conveyor).
- **Why**: Inelastic contact dissipates kinetic energy through impulses.
  Larger impulses should correlate with larger energy changes. Validates
  both the contact reporting pipeline and the energy computation.
- **Concession**: Conveyor uses a relaxed threshold (0.6) because sustained
  sliding contact produces weaker impulse-KE correlation than transient
  impacts.

## Consequences

- All five families run per scenario, per IC-grid run (27 runs × up to
  5 assertions = up to 135 assertion results).
- `validate_all.py` aggregates results into a per-run pass/fail table.
- `std(mass)` is effectively 0 for all runs (verified — mass never changes
  in a rigid-body simulation).
- The contact-energy Pearson r threshold may need further relaxation for
  conveyor runs with very slow belts (v_belt=0.2 → weaker sliding
  dynamics).
- These tests are explicitly NOT stdlib unittest — they require the PhysX
  solver and an existing per-run HDF5. Pure-logic tests (58 at time of
  writing) cover the computation modules under system python.
