# ADR-0007: Contact reporting via PhysxContactReportAPI subscription

## Status
Accepted — 2026-07-04

## Context
The dataset needs to distinguish conservative free-flight episodes from
dissipative contact episodes. This requires:
1. A per-frame `in_contact` boolean in `/state`
2. A ragged `/contacts` group with per-contact-point impulse, normal,
   and position data for contact-energy-agreement validation.

PhysX provides two contact reporting APIs:
- **Subscription** — `subscribe_contact_report_events(callback)` fires
  after each physics substep (4 times per capture frame at 480 Hz / 120 fps
  × 4 substeps). Includes impulse data even during impacts.
- **Immediate (pull)** — `get_contact_report()` returns contacts from only
  the last substep. For resting contacts, impulse is typically zero.

## Decision
Use the **subscription-based** `subscribe_contact_report_events` to capture
all contact events per frame, including impact impulses.

### Architecture

1. `common/contacts.py`:
   - `enable_contact_reporting(rigid_paths, threshold)` — applies
     `PhysxSchema.PhysxContactReportAPI` to each body prim before play.
   - `subscribe_contacts(callback)` — wraps the subscription.
   - `decode_contacts(headers, data)` — decodes `ContactEventHeaderVector`
     and `ContactDataVector` into Python dicts.
   - `has_active_contacts(contacts)` — True when Found or Persist events exist.
   - `set_kinematic_pairs_flags(physics_scene_path)` — sets
     `ReportKinematicKinematicPairs` and `ReportKinematicStaticPairs` on the
     PhysicsScene perm (not used by freefall; gated for conveyor in Slice F).

2. `common/loop.py` — subscribes to contacts, accumulates events per
   substep, flushes them as per-frame data after each `app.update()`.

3. `common/dataset_writer.py` — stores `in_contact` bool in `/state` and
   ragged `/contacts` as a structured array with step, position, normal,
   impulse, separation, face indices.

### Per-frame contact semantics
`in_contact` at frame `i` means: contacts were active during the physics
substeps that transitioned the body from frame `i-1` to frame `i`. This is
a "backward-looking" assignment (contacts from the step leading to the
current state), achieved by buffering contacts across substeps and flushing
after each `simulation_app.update()`.

## Consequences
- All scenario scripts that use `loop.run_capture_loop` automatically get
  contact reporting when `dataset_writer` is provided.
- The subscription object must be kept alive (assigned to a variable) or
  the carb subscription will be garbage-collected and stop firing.
- Kinematic pair flags are opt-in via `set_kinematic_pairs_flags`; not
  enabled by default (freefall doesn't need them).
- Contact data volume scales with substep count (4× per frame); for
  sustained contacts this produces many data points per frame.
- `in_contact` stays True for the entire ground-contact episode (no
  False gaps between bounces), so contact-window analysis must use
  per-frame impulse data rather than the binary `in_contact` flag.
