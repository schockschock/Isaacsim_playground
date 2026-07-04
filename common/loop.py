"""Shared simulation capture loop.

Isaac-Sim-bound counterpart to the pure helpers in common.stability: drives
the timeline, captures one frame per step via rep.orchestrator, reads back
the rigid-body state, and stops when the body settles or MAX_FRAMES is hit.
Camera-count-agnostic: a stereo rig's two render products are both captured
on every rep.orchestrator.step because the writer is attached to both.
"""
import time

import numpy as np
import omni.replicator.core as rep
import omni.timeline

from common.asset import read_rigid_body_state
from common.stability import is_settled

RT_SUBFRAMES = 1
WARMUP_FRAMES = 5
PROGRESS_EVERY = 30


def _enrich_state(state, m, prev_e_total):
    from common.measurements import (body_inertia_from_column_major,
                                     world_inertia, kinetic_energies,
                                     potential_energy, momenta)

    I_body_9 = state.pop("inertia_body_col9")
    com_offset = state.pop("com_offset")

    I_body = body_inertia_from_column_major(I_body_9)
    q = state["orientation"]
    I_world = world_inertia(I_body, q)
    ke_trans, ke_rot = kinetic_energies(m, state["lin_velocity"], I_world,
                                          state["ang_velocity"])
    pe = potential_energy(m, state["position"][2])
    p_lin, L_world, L_body = momenta(m, state["lin_velocity"], I_world,
                                       state["ang_velocity"], I_body)
    e_total = ke_trans + ke_rot + pe

    if prev_e_total is None:
        dissipating = False
    else:
        dissipating = abs(e_total - prev_e_total) > 1e-3

    state["mass"] = m
    state["inertia_world"] = I_world
    state["E_trans"] = ke_trans
    state["E_rot"] = ke_rot
    state["PE_grav"] = pe
    state["E_total"] = e_total
    state["p_lin"] = np.array(p_lin, dtype=np.float64)
    state["L_ang_world"] = np.array(L_world, dtype=np.float64)
    state["L_ang_body"] = np.array(L_body, dtype=np.float64)
    state["dissipating"] = dissipating

    return state, I_body_9, com_offset, e_total


def run_capture_loop(simulation_app, prim, max_frames, dt,
                     stability_window, stability_epsilon,
                     rt_subframes=RT_SUBFRAMES, warmup_frames=WARMUP_FRAMES,
                     progress_every=PROGRESS_EVERY,
                     dataset_writer=None, rb_view=None):
    """Run the capture loop for a single rigid body until settled or max frames.

    Expects the timeline to be stopped and the scene fully built. Starts the
    timeline, warms up, then captures one frame per iteration. On each frame:
      1. rep.orchestrator.step(delta_time=0.0, rt_subframes, pause_timeline=False)
         — captures the current state without advancing the timeline.
      2. read back position via common.asset.read_rigid_body_state and feed a
         sliding window to common.stability.is_settled.
      3. simulation_app.update() — advances the timeline by one frame
         (PhysX runs 4 substeps at 480 Hz / 120 fps).

    When dataset_writer and rb_view are provided, per-frame state is logged
    to an HDF5 file via the Tensor API (common.readback), including during
    warmup. The HDF5 attr ``t_offset_frames`` records how many state rows
    were logged before the first RGB frame.

    Args:
        simulation_app: the SimulationApp instance.
        prim: rigid body USD prim to monitor for settling.
        max_frames: hard cap on captured frames.
        dt: capture timestep in seconds (1/stage_rate); used only for logging.
        stability_window: consecutive frames required to call it settled.
        stability_epsilon: max per-axis drift (m) over the window.
        rt_subframes: rt subframes per capture (1 for PathTracing at SPP).
        warmup_frames: timeline updates before the capture loop starts.
        progress_every: print a progress line every N frames.
        dataset_writer: optional DatasetWriter for HDF5 state logging.
        rb_view: optional RigidPrim from common.readback.setup_rigid_body_view.

    Returns:
        (frames_captured, settled: bool).
    """
    from common.readback import read_full_state as tensor_read_full_state
    from common.readback import read_state as tensor_read_state
    from common.contacts import decode_contacts, has_active_contacts, subscribe_contacts

    timeline = omni.timeline.get_timeline_interface()
    timeline.set_looping(False)
    if not timeline.is_playing():
        timeline.play()

    state_row_offset = 0
    prev_e_total = None
    invariants_set = False
    pending_in_contact = False
    pending_contacts = []
    contact_buffer = []

    def _on_contact(contact_headers, contact_data):
        decoded = decode_contacts(contact_headers, contact_data)
        contact_buffer.extend(decoded)

    if dataset_writer is not None:
        _contact_sub = subscribe_contacts(_on_contact)

    for _ in range(warmup_frames):
        if dataset_writer is not None and rb_view is not None:
            state = tensor_read_full_state(rb_view)
            m = state["mass"]
            state, I_body_9, com_offset, prev_e_total = _enrich_state(state, m, prev_e_total)
            state["time"] = float(state_row_offset) * dt
            state["in_contact"] = pending_in_contact
            if not invariants_set:
                dataset_writer.set_body_invariants(com_offset, I_body_9)
                invariants_set = True
            dataset_writer.append_state(state)
            dataset_writer.append_contacts(state_row_offset, pending_contacts)
            state_row_offset += 1
        simulation_app.update()
        if dataset_writer is not None:
            pending_contacts = list(contact_buffer)
            contact_buffer.clear()
            pending_in_contact = has_active_contacts(pending_contacts)

    if dataset_writer is not None:
        dataset_writer._file.attrs["t_offset_frames"] = warmup_frames

    print(f"\n  Starting capture loop ({max_frames} max frames, "
          f"stability: {stability_window} frames @ {stability_epsilon}m)...")

    pos_window = []
    settled = False
    wall_start = time.perf_counter()

    for frame_idx in range(max_frames):
        rep.orchestrator.step(
            delta_time=0.0,
            rt_subframes=rt_subframes,
            pause_timeline=False,
        )

        pos, _quat, lv, av = read_rigid_body_state(prim)
        pos_window.append((float(pos[0]), float(pos[1]), float(pos[2])))
        if len(pos_window) > stability_window:
            pos_window.pop(0)

        if dataset_writer is not None and rb_view is not None:
            state = tensor_read_full_state(rb_view)
            m = state["mass"]
            state, I_body_9, com_offset, prev_e_total = _enrich_state(state, m, prev_e_total)
            state["time"] = float(state_row_offset) * dt
            state["in_contact"] = pending_in_contact
            if not invariants_set:
                dataset_writer.set_body_invariants(com_offset, I_body_9)
                invariants_set = True
            dataset_writer.append_state(state)
            dataset_writer.append_contacts(state_row_offset, pending_contacts)
            state_row_offset += 1

        if is_settled(pos_window, stability_epsilon, stability_window):
            settled = True
            print(f"  Settled at frame {frame_idx} ({frame_idx * dt:.3f}s)")
            break

        if frame_idx % progress_every == 0:
            speed = float(lv[0]) ** 2 + float(lv[1]) ** 2 + float(lv[2]) ** 2
            speed = speed ** 0.5
            ang_speed = float(av[0]) ** 2 + float(av[1]) ** 2 + float(av[2]) ** 2
            ang_speed = ang_speed ** 0.5
            print(f"  frame {frame_idx:3d}/{max_frames}  "
                  f"pos=({float(pos[0]):.3f},{float(pos[1]):.3f},{float(pos[2]):.3f})  "
                  f"|v|={speed:.3f}m/s  |w|={ang_speed:.1f}rad/s")

        simulation_app.update()
        if dataset_writer is not None:
            pending_contacts = list(contact_buffer)
            contact_buffer.clear()
            pending_in_contact = has_active_contacts(pending_contacts)

    if dataset_writer is not None:
        dataset_writer.set_n_rgb_frames(frame_idx + 1)

    elapsed = time.perf_counter() - wall_start
    print(f"  Capture done: {frame_idx + 1} frames in {elapsed:.1f}s "
          f"(settled={settled})")

    rep.orchestrator.wait_until_complete()
    return frame_idx + 1, settled


def teardown(writer, render_products):
    """Detach writer and destroy render products."""
    writer.detach()
    for rp in render_products:
        rp.destroy()
