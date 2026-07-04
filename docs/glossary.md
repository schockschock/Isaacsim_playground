# Glossary

Domain terminology for the Isaac Sim Playground. The authoritative glossary is
in `CONTEXT.md` §Glossary. This file adds terms introduced after the initial
project setup.

## ground-truth state
The PhysX solver's internal kinematic state read back via the Tensor API
(see `common/readback.py`, ADR-0006): position, orientation (wxyz), linear
velocity (m/s), and angular velocity (rad/s) of a rigid body's center of mass,
in world coordinates. No finite differences. Extended in Slice B to include
mass, COM offset, and inertia tensor.

## PhysX Tensor API
`omni.physics.tensors` — the low-level Python binding that reads/writes PhysX
solver buffers directly via warp, numpy, or torch frontends. In this project,
accessed through the higher-level `isaacsim.core.experimental.prims.RigidPrim`
wrapper which uses the warp frontend internally. Reports all quantities in SI
units (m, m/s, rad/s, kg).

## metersPerUnit
A USD stage metadata value (`stage.GetMetersPerUnit()`) that defines the
conversion between USD scene units and real-world meters. Isaac Sim's default
is 0.01 (1 scene unit = 1 cm). The Tensor API reports values in scene units,
which must be multiplied by `metersPerUnit` to convert to meters. However,
RigidPrim already applies this conversion internally, so all values from
`readback.read_state()` are in real meters.

## DatasetWriter
`common/dataset_writer.DatasetWriter` — the per-run HDF5 writer (ADR-0005).
Buffer-per-frame state rows and writes them to `<scenario>.h5` alongside run
metadata attributes. Pure h5py + numpy, no Isaac deps.

## t_offset_frames
Number of physics steps logged to the HDF5 before the first RGB frame was
captured (the warmup period). Allows downstream consumers to align state rows
with RGB frame indices. Stored as an HDF5 attribute.

## RigidPrim
High-level rigid-body wrapper from `isaacsim.core.experimental.prims`. Handles
Tensor API initialization via `SimulationManager.initialize_physics()` and
exposes getters for world poses, velocities, masses, COM, and inertias.
Used by `common.readback.py` for all ground-truth state readback.

## PhysxContactReportAPI
A PhysX schema API (`PhysxSchema.PhysxContactReportAPI`) that, when applied to
a rigid body prim, causes PhysX to emit per-substep contact reports via
`subscribe_contact_report_events`. Configured with a force threshold (0.0 =
report all). Applied in `common/contacts.py:enable_contact_reporting`.

## in_contact
A per-frame bool in `/state` indicating whether contact events (Found or
Persist) occurred during the physics substeps leading to that frame. Set
via the contact subscription in `common/loop.py`. True for the entire
contact episode; individual impacts are identified by non-zero impulse.

## contact impulse
The `ContactData.impulse` field from PhysX contact reports — the change in
momentum (force × dt) at a single contact point during one physics substep.
Units: N·s per substep. Summed per frame for the `/contacts` HDF5 group.

## contact-energy agreement
An integration-test assertion (validate_freefall.py) that checks whether the
magnitude of kinetic energy change across contact frames correlates with the
sum of contact impulse magnitudes (Pearson r). Physically: larger impulses
dissipate more kinetic energy. Current threshold: r > 0.7.

## parallel-shift stereo
A rectified stereo rig where both cameras share the same forward direction
(parallel optical axes), offset horizontally by the baseline. No keystone
distortion, zero vertical disparity. Enables `d = f·B/Z` directly without
rectification. Introduced in ADR-0004, superseding the toed-in default of
ADR-0002.

## ground-truth modality
A per-frame annotation emitted by BasicWriter alongside RGB: depth
(`distance_to_image_plane`), semantic/instance segmentation, motion vectors,
normals, bounding boxes, and camera intrinsics. All enabled in `setup_writer`
with `colorize_*=False` for raw uint32/float32 data.
