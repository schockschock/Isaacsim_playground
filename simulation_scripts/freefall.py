"""freefall — potato dropped onto the gridroom ground (compliant, stereo).

Compliant version of samples/freefall.py: headless, PathTracing, parametrized
via top-of-file constants, PNG + MP4 output, shared logic via common/.
Legacy samples/freefall.py is left untouched as a learning artifact.

Run:  conda deactivate && $ISAAC_SIM/python.sh simulation_scripts/freefall.py
      conda deactivate && $ISAAC_SIM/python.sh simulation_scripts/freefall.py --run V P
"""
import argparse
import math
import os
import sys

# --- Parse args before SimulationApp ---
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
if MATRIX_MODE:
    VEL_IDX, POSE_IDX = _args.run
else:
    VEL_IDX, POSE_IDX = None, None
OBJECT_ID = _args.object
OUTPUT_BASE = _args.output_base

from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": True})

import omni.replicator.core as rep
from pxr import Gf, UsdPhysics

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
_P0 = (1.0, 0.0, 0.0, 0.0)            # rest orientation
_P1 = (0.924, 0.383, 0.0, 0.0)         # 45 deg tilt about X
_P2 = (0.5, 0.5, 0.5, 0.5)            # 120 deg about (1,1,1)
_IC_POSES = [_P0, _P1, _P2]

# Velocity axis (0.3, 0.5, 1.0) normalized
_axis_norm = math.sqrt(0.3**2 + 0.5**2 + 1.0**2)
_AXIS = (0.3 / _axis_norm, 0.5 / _axis_norm, 1.0 / _axis_norm)
_V0 = (0.0, 0.0, 0.0)
_V1 = tuple(w * 10.0 for w in _AXIS)
_V2 = tuple(w * 20.0 for w in _AXIS)
_IC_ANGULAR_VELS = [_V0, _V1, _V2]

# --- Initial conditions (defaults; overridden by --run) ---
INIT_POSITION = (0.0, 0.0, 0.4)
INIT_QUATERNION = (1.0, 1.0, 0.0, 0.0)   # (w, x, y, z)
INIT_LINEAR_VELOCITY = (0.0, 0.0, 0.0)   # m/s — dropped from rest
INIT_ANGULAR_VELOCITY = (20.0, 5.5, 5.0)  # rad/s (freefall default)

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

SCENARIO = "freefall"
OUTPUT_SUBDIR = "freefall_high_angular_velocity"  # default; overridden in --run mode


def _apply_ic():
    global INIT_QUATERNION, INIT_ANGULAR_VELOCITY, OUTPUT_SUBDIR, USD_PATH
    if DATASET_MODE:
        INIT_QUATERNION = _IC_POSES[POSE_IDX] if MATRIX_MODE else _IC_POSES[0]
        INIT_ANGULAR_VELOCITY = (_IC_ANGULAR_VELS[VEL_IDX] if MATRIX_MODE
                                  else _IC_ANGULAR_VELS[0])
        v_tag = f"v{VEL_IDX}_p{POSE_IDX}" if MATRIX_MODE else "v0_p0"
        OUTPUT_SUBDIR = f"{SCENARIO}/{OBJECT_ID}/{v_tag}"
        USD_PATH = (f"/data2/adrien/clean_dataset/{OBJECT_ID}"
                     f"/{OBJECT_ID}_centered.usd")
    elif MATRIX_MODE:
        INIT_QUATERNION = _IC_POSES[POSE_IDX]
        INIT_ANGULAR_VELOCITY = _IC_ANGULAR_VELS[VEL_IDX]
        OUTPUT_SUBDIR = f"freefall/v{VEL_IDX}_p{POSE_IDX}"


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
    # BasicWriter lays out per-eye output as <RP_name>/rgb/rgb_%04d.png
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
