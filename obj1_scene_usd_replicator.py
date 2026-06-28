#!/usr/bin/env python3
"""Objective 1 — Load scene, load potato USD, replicator photos.

Run:  $ISAAC_SIM/python.sh obj1_scene_usd_replicator.py
"""

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": True})

import omni.replicator.core as rep
import omni.timeline
import omni.usd

from common import asset, replicator, world

# --- Configure replicator ---
rep.set_global_seed(42)
rep.orchestrator.set_capture_on_play(False)

# --- Build scene ---
print("[obj1] Setting up world...")
stage = world.setup()

print("[obj1] Loading potato asset...")
potato = asset.load(name="Potato", position=(0, 0, 1.2))
asset.add_colliders(potato)
asset.make_rigid(potato)

# --- Camera + writer ---
print("[obj1] Setting up camera...")
cam = replicator.setup_camera(name="MainCam", parent="/World")
rp = rep.create.render_product(cam.GetPath(), (1280, 720), name="MainCamRenderProduct")
rp.hydra_texture.set_updates_enabled(False)

writer, out_dir = replicator.setup_writer("obj1")
writer.attach(rp)

# --- Timeline ---
timeline = omni.timeline.get_timeline_interface()
timeline.set_start_time(0)
timeline.set_end_time(1000000)
timeline.set_looping(False)
timeline.play()
timeline.commit()
simulation_app.update()

# --- Settle physics ---
print("[obj1] Settling physics...")
for _ in range(30):
    simulation_app.update()

# --- Capture frames ---
NUM_FRAMES = 30
STEPS_BETWEEN_CAPTURES = 10
RT_SUBFRAMES = 8
print(f"[obj1] Capturing {NUM_FRAMES} frames...")
for i in range(NUM_FRAMES):
    rp.hydra_texture.set_updates_enabled(True)
    rep.orchestrator.step(delta_time=0.0,
                          rt_subframes=RT_SUBFRAMES,
                          pause_timeline=False)
    rp.hydra_texture.set_updates_enabled(False)
    if i % 5 == 0:
        print(f"  frame {i}/{NUM_FRAMES}")
    for _ in range(STEPS_BETWEEN_CAPTURES):
        simulation_app.update()

# --- Teardown ---
rep.orchestrator.wait_until_complete()
timeline.stop()
simulation_app.update()
simulation_app.close()
print(f"[obj1] Done. Photos written to {out_dir}")
