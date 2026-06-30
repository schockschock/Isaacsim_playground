"""colliders — three potatoes with different collider approximations, side view.

Three potato prims dropped side-by-side, each with a different
UsdPhysics.MeshCollisionAPI approximation (convexHull, convexDecomposition,
sdf), identical drop height and material. Side-view stereo rig looks along
the row so all three prims are visible with parallax. Otherwise reuses the
freefall flow and shared capture loop.

Run:  conda deactivate && $ISAAC_SIM/python.sh samples/simulations/colliders.py
"""
import os

from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": True})

import omni.replicator.core as rep
from pxr import Gf, UsdPhysics

from common.app import shutdown
from common.world import setup_stage
from common.asset import load, add_colliders, apply_physics_material
from common.replicator import setup_stereo_rig
from common.loop import run_capture_loop, teardown
from common.video import pngs_to_mp4, stereo_pair_to_mp4

# --- Asset ---
USD_PATH = "/data2/adrien/clean_dataset/2R1-1/2R1-1_centered.usd"

# --- Three prims side-by-side along X ---
APPROXIMATIONS = ["convexHull", "convexDecomposition", "sdf"]
PRIM_NAMES = ["Potato_" + a for a in APPROXIMATIONS]
X_OFFSETS = [-0.15, 0.0, 0.15]
DROP_HEIGHT = 0.4
INIT_QUATERNION = (1.0, 1.0, 0.0, 0.0)
INIT_ANGULAR_VELOCITY = (0.0, 0.5, 5.0)   # identical spin on all three

# --- Object material & mass (identical across the three) ---
OBJECT_DENSITY = 1000.0
OBJECT_RESTITUTION = 0.35
OBJECT_STATIC_FRICTION = 0.6
OBJECT_DYNAMIC_FRICTION = 0.6

# --- PhysX / capture rates ---
DT = 1.0 / 120.0
MAX_FRAMES = 100
STABILITY_WINDOW = 5
STABILITY_EPSILON = 1e-4

# --- Stereo rig (side view along Y, baseline along X = the row axis) ---
CAM_BASE = (0.0, 1.2, 0.20)              # off the +Y side, looking at the row
CAM_LOOK_AT = (0.0, 0.0, 0.10)
CAM_FOCAL_LENGTH = 24.0
CAM_RESOLUTION = (720, 720)
STEREO_BASELINE = 0.06

OUTPUT_SUBDIR = "colliders"


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

    # --- Three potatoes, one per approximation ---
    init_quat = Gf.Quatd(
        INIT_QUATERNION[0],
        Gf.Vec3d(INIT_QUATERNION[1], INIT_QUATERNION[2], INIT_QUATERNION[3]),
    )
    prims = []
    for name, x, approx in zip(PRIM_NAMES, X_OFFSETS, APPROXIMATIONS):
        prim = load(
            name=name, prim_path=USD_PATH,
            position=(x, 0.0, DROP_HEIGHT), rotation=init_quat,
        )
        rep.functional.physics.apply_rigid_body(prim, disableGravity=False)
        mass_api = UsdPhysics.MassAPI.Apply(prim)
        mass_api.CreateDensityAttr(OBJECT_DENSITY)
        add_colliders(prim, approximation=approx)
        apply_physics_material(
            prim,
            restitution=OBJECT_RESTITUTION,
            static_friction=OBJECT_STATIC_FRICTION,
            dynamic_friction=OBJECT_DYNAMIC_FRICTION,
        )
        prim.GetAttribute("physics:angularVelocity").Set(Gf.Vec3f(*INIT_ANGULAR_VELOCITY))
        prims.append(prim)

    # Monitor the middle prim for settling; MAX_FRAMES caps the rest.
    monitor = prims[1]

    frames, settled = run_capture_loop(
        simulation_app, monitor,
        max_frames=MAX_FRAMES, dt=DT,
        stability_window=STABILITY_WINDOW,
        stability_epsilon=STABILITY_EPSILON,
    )

    teardown(writer, rps)

    # --- MP4 (BasicWriter layout: <RP_name>/rgb/rgb_%04d.png) ---
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
