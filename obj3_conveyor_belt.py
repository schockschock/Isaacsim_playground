#!/usr/bin/env python3
"""Objective 3 — Conveyor belt spawning potato + replicator photos.

Run:  $ISAAC_SIM/python.sh obj3_conveyor_belt.py
"""

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": True})

import omni.replicator.core as rep
import omni.timeline
import omni.usd
from pxr import Gf, PhysxSchema, UsdGeom, UsdPhysics

from common import asset, replicator, world

# --- Replicator config ---
rep.set_global_seed(42)
rep.orchestrator.set_capture_on_play(False)

# --- Build scene ---
print("[obj3] Setting up world...")
stage = world.setup()

# --- Conveyor belt surface ---
BELT_SIZE_X = 4.0
BELT_SIZE_Y = 1.0
BELT_SIZE_Z = 0.1
BELT_HEIGHT = 0.5
BELT_SPEED = -0.5

print(f"[obj3] Creating conveyor belt (size={BELT_SIZE_X}x{BELT_SIZE_Y}m, speed={BELT_SPEED}m/s)...")
belt_path = "/World/ConveyorBelt"
belt_prim = UsdGeom.Cube.Define(stage, belt_path).GetPrim()

belt_xform = UsdGeom.XformCommonAPI(belt_prim)
belt_xform.SetScale((BELT_SIZE_X / 2, BELT_SIZE_Y / 2, BELT_SIZE_Z / 2))
belt_xform.SetTranslate((0, 0, BELT_HEIGHT))

UsdPhysics.CollisionAPI.Apply(belt_prim)
rigid = UsdPhysics.RigidBodyAPI.Apply(belt_prim)
rigid.CreateKinematicEnabledAttr(True)

vel_api = PhysxSchema.PhysxSurfaceVelocityAPI.Apply(belt_prim)
vel_api.CreateSurfaceVelocityAttr().Set(Gf.Vec3f(BELT_SPEED, 0, 0))

# --- Spawn potato on belt ---
print("[obj3] Spawning potato on conveyor belt...")
potato = asset.load(name="Potato", position=(0, 0, BELT_HEIGHT + BELT_SIZE_Z / 2 + 0.05))
asset.add_colliders(potato, approximation="convexHull")
asset.make_rigid(potato)

# --- Camera ---
print("[obj3] Setting up camera...")
cam = replicator.setup_camera(name="TopCam", parent="/World")
cam_path = cam.GetPath()

rp = rep.create.render_product(cam_path, (1920, 1080))
rp.hydra_texture.set_updates_enabled(False)

writer, out_dir = replicator.setup_writer("obj3")
writer.attach([rp])

# --- Timeline ---
timeline = omni.timeline.get_timeline_interface()
timeline.set_start_time(0)
timeline.set_end_time(1000000)
timeline.set_looping(False)
timeline.play()
timeline.commit()
simulation_app.update()

# --- Settle ---
print("[obj3] Settling...")
for _ in range(20):
    simulation_app.update()

# --- Capture frames ---
NUM_FRAMES = 60
STEPS_BETWEEN_CAPTURES = 1
RT_SUBFRAMES = 8
print(f"[obj3] Capturing {NUM_FRAMES} frames...")
for i in range(NUM_FRAMES):
    rp.hydra_texture.set_updates_enabled(True)
    rep.orchestrator.step(delta_time=0.0, rt_subframes=RT_SUBFRAMES, pause_timeline=False)
    rp.hydra_texture.set_updates_enabled(False)
    if i % 10 == 0:
        print(f"  frame {i}/{NUM_FRAMES}")
    for _ in range(STEPS_BETWEEN_CAPTURES):
        simulation_app.update()

# --- Teardown ---
rep.orchestrator.wait_until_complete()
timeline.stop()
simulation_app.update()
simulation_app.close()
print(f"[obj3] Done. Photos in {out_dir}/")
