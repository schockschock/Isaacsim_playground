"""object_throw — potato launched sideways, rebounds off a wall then the floor.

The potato is thrown from one side with a tunable initial linear + angular
velocity, rebounds off a vertical wall, then off the ground. A static wall
prim (box collider, no rigid body = immovable) sits across the path. A
wide stereo rig frames the full trajectory (wall + floor impact zones).
All knobs are top-of-file constants.

Run:  conda deactivate && $ISAAC_SIM/python.sh simulation_scripts/object_throw.py
      conda deactivate && $ISAAC_SIM/python.sh simulation_scripts/object_throw.py --run V P
"""
import argparse
import math
import os
import sys

_RUN_HELP = "Matrix mode: --run <velocity_level> <pose_index>"
_OBJ_HELP = "Dataset mode: --object <object_id>"
_OUT_HELP = "Output root dir for --object mode"
_parser = argparse.ArgumentParser(add_help=False)
_parser.add_argument("--run", type=int, nargs=2, metavar=("V", "P"), help=_RUN_HELP)
_parser.add_argument("--object", type=str, default=None, metavar="ID", help=_OBJ_HELP)
_parser.add_argument("--output-base", type=str,
                     default="/data2/adrien/physXdataset", metavar="DIR", help=_OUT_HELP)
_args, _ = _parser.parse_known_args()
MATRIX_MODE = _args.run is not None
DATASET_MODE = _args.object is not None
VEL_IDX, POSE_IDX = (_args.run if MATRIX_MODE else (None, None))
OBJECT_ID = _args.object
OUTPUT_BASE = _args.output_base

from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": True})

import omni.replicator.core as rep
from pxr import Gf, UsdGeom, UsdPhysics

from common.app import shutdown
from common.world import setup_stage
from common.asset import load, add_colliders, apply_physics_material, set_initial_state
from common.contacts import enable_contact_reporting
from common.dataset_writer import DatasetWriter
from common.readback import setup_rigid_body_view
from common.replicator import setup_stereo_rig
from common.loop import run_capture_loop, teardown
from common.video import pngs_to_mp4, stereo_pair_to_mp4
from common.ground_truth import post_process_disparity

# --- Asset ---
USD_PATH = "/data2/adrien/clean_dataset/2R1-1/2R1-1_centered.usd"

# --- 27-run IC grid (ADR-0008) ---
_P0 = (1.0, 0.0, 0.0, 0.0)
_P1 = (0.924, 0.383, 0.0, 0.0)
_P2 = (0.5, 0.5, 0.5, 0.5)
_IC_POSES = [_P0, _P1, _P2]

# Velocity direction (1, 0, 0.1) normalized
_dir_norm = math.sqrt(1.0**2 + 0.1**2)
_DIR = (1.0 / _dir_norm, 0.0, 0.1 / _dir_norm)
_V_SCALE = [2.0, 5.0, 9.0]
_OMEGA = (0.0, 0.0, 8.0)  # about Z axis

# --- Initial conditions (defaults; overridden by --run) ---
INIT_POSITION = (0.0, 0.0, 0.4)
INIT_QUATERNION = (1.0, 1.0, 0.0, 0.0)
INIT_LINEAR_VELOCITY = (10.0, 0.0, 1.0)    # m/s
INIT_ANGULAR_VELOCITY = (0.0, 0.0, 8.0)   # rad/s

# --- Wall (static collider across the +X side of the path) ---
WALL_PATH = "/World/Wall"
WALL_POSITION = (0.8, 0.0, 0.5)           # center; height 1.0 -> spans z=0..1.0
WALL_THICKNESS = 0.2                       # X extent (m)
WALL_HEIGHT = 1.0                          # Z extent (m)
WALL_WIDTH = 2.0                           # Y extent (m)

# --- Object material & mass (locked fixed material) ---
OBJECT_DENSITY = 1000.0
OBJECT_RESTITUTION = 0.4
OBJECT_STATIC_FRICTION = 0.5
OBJECT_DYNAMIC_FRICTION = 0.5

# --- PhysX / capture rates ---
DT = 1.0 / 120.0
MAX_FRAMES = 150                          # longer — trajectory takes time
STABILITY_WINDOW = 5
STABILITY_EPSILON = 1e-4

# --- Stereo rig (wide framing, baseline perpendicular to trajectory plane) ---
CAM_BASE = (0.1, 2.5, 0.6)
CAM_LOOK_AT = (0.1, 0.0, 0.3)
CAM_FOCAL_LENGTH = 24.0
CAM_RESOLUTION = (720, 720)
STEREO_BASELINE = 0.06

SCENARIO = "object_throw"
OUTPUT_SUBDIR = "object_throw"


def _apply_ic():
    global INIT_QUATERNION, INIT_LINEAR_VELOCITY, INIT_ANGULAR_VELOCITY, OUTPUT_SUBDIR, USD_PATH
    if DATASET_MODE:
        INIT_QUATERNION = (_IC_POSES[POSE_IDX] if MATRIX_MODE
                            else _IC_POSES[0])
        if MATRIX_MODE:
            v = _V_SCALE[VEL_IDX]
            INIT_LINEAR_VELOCITY = (v * _DIR[0], v * _DIR[1], v * _DIR[2])
        INIT_ANGULAR_VELOCITY = _OMEGA
        v_tag = f"v{VEL_IDX}_p{POSE_IDX}" if MATRIX_MODE else "v0_p0"
        OUTPUT_SUBDIR = f"{SCENARIO}/{OBJECT_ID}/{v_tag}"
        USD_PATH = (f"/data2/adrien/clean_dataset/{OBJECT_ID}"
                     f"/{OBJECT_ID}_centered.usd")
    elif MATRIX_MODE:
        INIT_QUATERNION = _IC_POSES[POSE_IDX]
        v = _V_SCALE[VEL_IDX]
        INIT_LINEAR_VELOCITY = (v * _DIR[0], v * _DIR[1], v * _DIR[2])
        INIT_ANGULAR_VELOCITY = _OMEGA
        OUTPUT_SUBDIR = f"object_throw/v{VEL_IDX}_p{POSE_IDX}"


def _add_wall(stage):
    """Add a static vertical wall: a box with CollisionAPI but no RigidBodyAPI,
    so PhysX treats it as an immovable collider.
    """
    wall = UsdGeom.Cube.Define(stage, WALL_PATH)
    wall.CreateSizeAttr(1.0)
    UsdGeom.XformCommonAPI(wall).SetScale((WALL_THICKNESS, WALL_WIDTH, WALL_HEIGHT))
    UsdGeom.XformCommonAPI(wall).SetTranslate(WALL_POSITION)
    UsdPhysics.CollisionAPI.Apply(wall.GetPrim())
    apply_physics_material(
        wall.GetPrim(),
        restitution=0.5,
        static_friction=0.6,
        dynamic_friction=0.6,
    )
    return wall.GetPrim()


def run_simulation() -> None:
    _apply_ic()
    stage = setup_stage()

    left_cam, right_cam, rps, writer = setup_stereo_rig(
        base_position=CAM_BASE,
        look_at=CAM_LOOK_AT,
        resolution=CAM_RESOLUTION,
        baseline=STEREO_BASELINE,
        focal_length=CAM_FOCAL_LENGTH,
        output_subdir=OUTPUT_SUBDIR,
        output_base=OUTPUT_BASE if DATASET_MODE else None,
        enable_semantic_segmentation=not DATASET_MODE,
    )

    _add_wall(stage)

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

    enable_contact_reporting(["/World/Potato"], threshold=0.0)

    # --- HDF5 output ---
    out_root = os.path.join(OUTPUT_BASE, OUTPUT_SUBDIR) if DATASET_MODE else os.path.join(
        os.path.dirname(os.path.dirname(__file__)), "_output", OUTPUT_SUBDIR)
    os.makedirs(out_root, exist_ok=True)
    if DATASET_MODE:
        v_tag = f"v{VEL_IDX}_p{POSE_IDX}" if MATRIX_MODE else "v0_p0"
        h5_name = f"{SCENARIO}_{OBJECT_ID}_{v_tag}.h5"
    else:
        run_tag = f"v{VEL_IDX}_p{POSE_IDX}" if MATRIX_MODE else SCENARIO
        h5_name = f"{SCENARIO}_{run_tag}.h5"
    h5_path = os.path.join(out_root, h5_name)
    rb_view = setup_rigid_body_view("/World/Potato")

    ds_writer = DatasetWriter(
        h5_path,
        scenario=SCENARIO,
        fps=int(round(1.0 / DT)),
        dt_phys=1.0 / 480,
        solver_substeps=4,
        baseline_m=STEREO_BASELINE,
        material={
            "restitution": OBJECT_RESTITUTION,
            "friction": OBJECT_STATIC_FRICTION,
            "density": OBJECT_DENSITY,
        },
        potato_usd_path=USD_PATH,
    )

    # --- Capture ---
    frames, settled = run_capture_loop(
        simulation_app, potato,
        max_frames=MAX_FRAMES, dt=DT,
        stability_window=STABILITY_WINDOW,
        stability_epsilon=STABILITY_EPSILON,
        dataset_writer=ds_writer,
        rb_view=rb_view,
    )

    ds_writer.finalize()
    print(f"  wrote {h5_path}")

    teardown(writer, rps)

    # --- MP4 (skip in dataset mode) ---
    left_dir = os.path.join(out_root, "Left", "rgb")
    right_dir = os.path.join(out_root, "Right", "rgb")
    fps = int(round(1.0 / DT))
    if not DATASET_MODE:
        if os.path.isdir(left_dir):
            pngs_to_mp4(left_dir, os.path.join(out_root, "left.mp4"), fps=fps)
            print(f"  wrote {out_root}/left.mp4")
        if os.path.isdir(right_dir):
            pngs_to_mp4(right_dir, os.path.join(out_root, "right.mp4"), fps=fps)
            print(f"  wrote {out_root}/right.mp4")
        if os.path.isdir(left_dir) and os.path.isdir(right_dir):
            stereo_pair_to_mp4(left_dir, right_dir, os.path.join(out_root, "stereo_sbs.mp4"), fps=fps)
            print(f"  wrote {out_root}/stereo_sbs.mp4")

    post_process_disparity(out_root, STEREO_BASELINE)


run_simulation()
shutdown(simulation_app)
