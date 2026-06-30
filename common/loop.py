"""Shared simulation capture loop.

Isaac-Sim-bound counterpart to the pure helpers in common.stability: drives
the timeline, captures one frame per step via rep.orchestrator, reads back
the rigid-body state, and stops when the body settles or MAX_FRAMES is hit.
Camera-count-agnostic: a stereo rig's two render products are both captured
on every rep.orchestrator.step because the writer is attached to both.
"""
import time

import omni.replicator.core as rep
import omni.timeline

from common.asset import read_rigid_body_state
from common.stability import is_settled

RT_SUBFRAMES = 1
WARMUP_FRAMES = 5
PROGRESS_EVERY = 30


def run_capture_loop(simulation_app, prim, max_frames, dt,
                     stability_window, stability_epsilon,
                     rt_subframes=RT_SUBFRAMES, warmup_frames=WARMUP_FRAMES,
                     progress_every=PROGRESS_EVERY):
    """Run the capture loop for a single rigid body until settled or max frames.

    Expects the timeline to be stopped and the scene fully built. Starts the
    timeline, warms up, then captures one frame per iteration. On each frame:
      1. rep.orchestrator.step(delta_time=0.0, rt_subframes, pause_timeline=False)
         — captures the current state without advancing the timeline.
      2. read back position via common.asset.read_rigid_body_state and feed a
         sliding window to common.stability.is_settled.
      3. simulation_app.update() — advances the timeline by one frame
         (PhysX runs 4 substeps at 480 Hz / 120 fps).

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

    Returns:
        (frames_captured, settled: bool).
    """
    timeline = omni.timeline.get_timeline_interface()
    timeline.set_looping(False)
    if not timeline.is_playing():
        timeline.play()

    for _ in range(warmup_frames):
        simulation_app.update()

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
                  f"|v|={speed:.3f}m/s  |w|={ang_speed:.1f}deg/s")

        simulation_app.update()

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
