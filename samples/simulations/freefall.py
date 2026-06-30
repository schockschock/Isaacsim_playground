"""freefall — potato dropped onto the gridroom ground (compliant, stereo).

Compliant version of samples/freefall.py: headless, PathTracing, parametrized
via top-of-file constants, PNG + MP4 output, shared logic via common/.
Legacy samples/freefall.py is left untouched as a learning artifact.

Run:  conda deactivate && $ISAAC_SIM/python.sh samples/simulations/freefall.py
"""
import os

from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": True})

import omni.replicator.core as rep
from pxr import Gf, UsdPhysics

from common.app import shutdown
from common.world import setup_stage
from common.asset import load, add_colliders, apply_physics_material, set_initial_state
from common.replicator import setup_stereo_rig
from common.loop import run_capture_loop, teardown
from common.video import pngs_to_mp4, stereo_pair_to_mp4

# --- Asset ---
USD_PATH = "/data2/adrien/clean_dataset/2R1-1/2R1-1_centered.usd"

# --- Initial conditions ---
INIT_POSITION = (0.0, 0.0, 0.4)
INIT_QUATERNION = (1.0, 1.0, 0.0, 0.0)   # (w, x, y, z)
INIT_LINEAR_VELOCITY = (0.0, 0.0, 0.0)   # m/s — dropped from rest
INIT_ANGULAR_VELOCITY = (20.0, 5.5, 5.0)  # rad/s (converted to deg/s for PhysX)

# --- Object material & mass ---
OBJECT_DENSITY = 1000.0                  # kg/m^3
OBJECT_RESTITUTION = 0.35
OBJECT_STATIC_FRICTION = 0.6
OBJECT_DYNAMIC_FRICTION = 0.6
OBJECT_CONTACT_OFFSET = 0.005

# --- PhysX / capture rates ---
DT = 1.0 / 120.0                         # capture timestep (120 fps)
MAX_FRAMES = 100
STABILITY_WINDOW = 5
STABILITY_EPSILON = 1e-4

# --- Stereo rig ---
CAM_BASE = (0.0, 0.5, 0.25)             # rig center, above and to +Y of impact
CAM_LOOK_AT = (0.0, 0.0, 0.10)          # convergence at landing zone
CAM_FOCAL_LENGTH = 24.0
CAM_RESOLUTION = (720, 720)
STEREO_BASELINE = 0.06                   # 6 cm

OUTPUT_SUBDIR = "freefall_high_angular_velocity"  # subdir of _output/ for this run


def run_simulation() -> None:
    stage = setup_stage()

    left_cam, right_cam, rps, writer = setup_stereo_rig(
        base_position=CAM_BASE,
        look_at=CAM_LOOK_AT,
        resolution=CAM_RESOLUTION,
        baseline=STEREO_BASELINE,
        focal_length=CAM_FOCAL_LENGTH,
        output_subdir=OUTPUT_SUBDIR,
    )

    # --- Potato ---
    init_quat = Gf.Quatd(
        INIT_QUATERNION[0],
        Gf.Vec3d(INIT_QUATERNION[1], INIT_QUATERNION[2], INIT_QUATERNION[3]),
    )
    potato = load(
        name="Potato", prim_path=USD_PATH, position=INIT_POSITION,
        rotation=init_quat,
    )
    rep.functional.physics.apply_rigid_body(potato, disableGravity=False)

    mass_api = UsdPhysics.MassAPI.Apply(potato)
    mass_api.CreateDensityAttr(OBJECT_DENSITY)

    add_colliders(potato, approximation="convexHull")
    apply_physics_material(
        potato,
        restitution=OBJECT_RESTITUTION,
        static_friction=OBJECT_STATIC_FRICTION,
        dynamic_friction=OBJECT_DYNAMIC_FRICTION,
    )

    set_initial_state(potato, INIT_LINEAR_VELOCITY, INIT_ANGULAR_VELOCITY)

    # --- Capture ---
    frames, settled = run_capture_loop(
        simulation_app, potato,
        max_frames=MAX_FRAMES, dt=DT,
        stability_window=STABILITY_WINDOW,
        stability_epsilon=STABILITY_EPSILON,
    )

    teardown(writer, rps)

    # --- MP4 ---
    # BasicWriter lays out per-eye output as <RP_name>/rgb/rgb_%04d.png
    # (one subdir per annotator). Point the encoder at the rgb/ subfolder.
    out_root = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(__file__))), "_output", OUTPUT_SUBDIR)
    left_dir = os.path.join(out_root, "Left", "rgb")
    right_dir = os.path.join(out_root, "Right", "rgb")
    fps = int(round(1.0 / DT))
    if os.path.isdir(left_dir):
        pngs_to_mp4(left_dir, os.path.join(out_root, "left.mp4"), fps=fps)
        print(f"  wrote {out_root}/left.mp4")
    if os.path.isdir(right_dir):
        pngs_to_mp4(right_dir, os.path.join(out_root, "right.mp4"), fps=fps)
        print(f"  wrote {out_root}/right.mp4")
    if os.path.isdir(left_dir) and os.path.isdir(right_dir):
        stereo_pair_to_mp4(left_dir, right_dir, os.path.join(out_root, "stereo_sbs.mp4"), fps=fps)
        print(f"  wrote {out_root}/stereo_sbs.mp4")


run_simulation()
shutdown(simulation_app)
