#!/usr/bin/env python3
"""Objective 2 — 4 collider types comparison with physics-synced capture + metrics.

Run:  $ISAAC_SIM/python.sh obj2_collider_comparison.py
"""

import csv
import os
import time

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": True})

import omni.physics.core
import omni.replicator.core as rep
import omni.timeline
import omni.usd
from pxr import Gf, UsdGeom

from common import asset, replicator, world

# --- Collider types and positions ---
COLLIDER_TYPES = ["meshSimplification", "convexHull", "convexDecomposition", "sdf"]
X_SPACING = 1.0
X_OFFSETS = [-(len(COLLIDER_TYPES) - 1) / 2.0 * X_SPACING + i * X_SPACING for i in range(len(COLLIDER_TYPES))]
DROP_HEIGHT = 1.2

# --- Replicator config ---
rep.set_global_seed(42)
rep.orchestrator.set_capture_on_play(False)

# --- Build scene ---
print("[obj2] Setting up world...")
stage = world.setup()

# --- Load 4 potato instances side-by-side ---
print("[obj2] Loading 4 potato instances...")
potatoes = []
for i, (ctype, x_off) in enumerate(zip(COLLIDER_TYPES, X_OFFSETS)):
    name = f"Potato_{ctype}"
    prim = asset.load(name=name, position=(x_off, 0, DROP_HEIGHT))
    print(f"  [{i+1}/4] {ctype} at x={x_off:.1f}")
    asset.add_colliders(prim, approximation=ctype)
    asset.make_rigid(prim)
    potatoes.append((ctype, prim))

# --- Camera: side-on view of all 4 objects ---
print("[obj2] Setting up camera...")
center_x = (X_OFFSETS[0] + X_OFFSETS[-1]) / 2.0
cam = replicator.setup_camera(name="SideCam", parent="/World", focal_length=18.0)
cam_path = cam.GetPath()

cam_xform = UsdGeom.XformCommonAPI(cam)
cam_xform.SetTranslate((center_x, -4.0, DROP_HEIGHT / 2))
cam_xform.SetRotate((0, 90, 0), UsdGeom.XformCommonAPI.RotationOrderXYZ)

rp = rep.create.render_product(cam_path, (1920, 1080))
rp.hydra_texture.set_updates_enabled(False)

writer, out_dir = replicator.setup_writer("obj2")
writer.attach([rp])

# --- Metrics storage ---
metrics_rows = []
step_counter = [0]
capture_flag = [0]

OUTPUT_CSV = os.path.join(out_dir, "metrics.csv")

CAPTURE_EVERY_N = 4
MAX_PHYSICS_STEPS = 3000
current_time = [0.0]

# --- Physics step callback ---
def on_physics_step(dt: float, ctx) -> None:
    step_counter[0] += 1
    current_time[0] += dt
    step = step_counter[0]
    row = {"step": step, "time": current_time[0]}

    for ctype, prim in potatoes:
        matrix = omni.usd.get_world_transform_matrix(prim)
        pos = matrix.ExtractTranslation()
        vel_attr = prim.GetAttribute("physics:velocity")
        vel_vec = vel_attr.Get() if vel_attr else Gf.Vec3f(0, 0, 0)
        vel_y = vel_vec[1] if isinstance(vel_vec, Gf.Vec3f) else 0.0

        row[f"{ctype}_pos_y"] = pos[1]
        row[f"{ctype}_vel_y"] = vel_y
        row[f"{ctype}_contact"] = 1 if abs(pos[1]) < 0.03 and abs(vel_y) < 0.3 else 0

    metrics_rows.append(row)

    if step % CAPTURE_EVERY_N == 0:
        capture_flag[0] = step


physics_sub = omni.physics.core.get_physics_simulation_interface().subscribe_physics_on_step_events(
    pre_step=False, order=0, on_update=on_physics_step,
)

# --- Timeline ---
timeline = omni.timeline.get_timeline_interface()
timeline.set_start_time(0)
timeline.set_end_time(1000000)
timeline.set_looping(False)
timeline.play()
timeline.commit()
simulation_app.update()

# --- Main loop ---
captured = 0
print(f"[obj2] Running {MAX_PHYSICS_STEPS} physics steps, capturing every {CAPTURE_EVERY_N}th...")
wall_start = time.perf_counter()

while step_counter[0] < MAX_PHYSICS_STEPS:
    simulation_app.update()
    if capture_flag[0] > 0:
        capture_flag[0] = 0
        rp.hydra_texture.set_updates_enabled(True)
        rep.orchestrator.step(delta_time=0.0, rt_subframes=8, pause_timeline=False)
        rp.hydra_texture.set_updates_enabled(False)
        captured += 1
        if captured % 50 == 0:
            print(f"  captured {captured} frames at step {step_counter[0]}")

wall_dur = time.perf_counter() - wall_start
print(f"[obj2] Done in {wall_dur:.1f}s. {captured} frames, {len(metrics_rows)} metrics rows.")

# --- Write metrics CSV ---
print(f"[obj2] Writing metrics to {OUTPUT_CSV}...")
if metrics_rows:
    fieldnames = metrics_rows[0].keys()
    with open(OUTPUT_CSV, "w", newline="") as f:
        writer_csv = csv.DictWriter(f, fieldnames=fieldnames)
        writer_csv.writeheader()
        writer_csv.writerows(metrics_rows)

# --- Teardown ---
physics_sub = None
rep.orchestrator.wait_until_complete()
timeline.stop()
simulation_app.update()
simulation_app.close()
print(f"[obj2] All done. Output in {out_dir}/")
