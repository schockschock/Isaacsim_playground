"""conveyor — potato transported by a conveyor belt then falling off the end.

Uses the isaacsim.asset.gen.conveyor extension's create_conveyor_belt() to
drive a kinematic belt prim via PhysxSurfaceVelocityAPI. The extension
auto-applies RigidBodyAPI/CollisionAPI/PhysxSurfaceVelocityAPI/MeshCollisionAPI
to the belt, so we do NOT re-apply them. The potato is placed on the belt,
transported, then falls off the end into a drop zone. Stereo rig frames the
belt + drop zone.

Run:  conda deactivate && $ISAAC_SIM/python.sh simulation_scripts/conveyor.py
      conda deactivate && $ISAAC_SIM/python.sh simulation_scripts/conveyor.py --run V P
"""
import argparse
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

import omni.kit.app
import omni.replicator.core as rep
import omni.usd
from pxr import Gf, UsdGeom, UsdPhysics

from common.app import shutdown
from common.world import setup_stage
from common.asset import load, add_colliders, apply_physics_material, set_initial_state
from common.contacts import enable_contact_reporting, set_kinematic_pairs_flags
from common.dataset_writer import DatasetWriter
from common.readback import setup_rigid_body_view
from common.replicator import setup_stereo_rig
from common.loop import run_capture_loop, teardown
from common.video import pngs_to_mp4, stereo_pair_to_mp4
from common.ground_truth import post_process_disparity


def _ensure_conveyor_extension():
    """The conveyor extension is not enabled by default; load it explicitly.
    Must be called before importing isaacsim.asset.gen.conveyor."""
    ext_id = "isaacsim.asset.gen.conveyor"
    mgr = omni.kit.app.get_app().get_extension_manager()
    if not mgr.is_extension_enabled(ext_id):
        mgr.set_extension_enabled_immediate(ext_id, True)

# --- Asset ---
USD_PATH = "/data2/adrien/clean_dataset/2R1-1/2R1-1_centered.usd"

# --- 27-run IC grid (ADR-0008) ---
_P0 = (1.0, 0.0, 0.0, 0.0)
_P1 = (0.924, 0.383, 0.0, 0.0)
_P2 = (0.5, 0.5, 0.5, 0.5)
_IC_POSES = [_P0, _P1, _P2]
_IC_BELT_VELS = [0.2, 0.5, 1.0]

# --- Conveyor belt (a long box, kinematic) ---
BELT_PATH = "/World/ConveyorBelt"
BELT_POSITION = (-0.2, 0.0, 0.05)
BELT_LENGTH = 1.1
BELT_WIDTH = 0.4
BELT_THICKNESS = 0.1
BELT_VELOCITY = 0.5     # default; overridden by --run
BELT_DIRECTION = (1.0, 0.0, 0.0)
BELT_FRICTION = 0.8
BELT_RESTITUTION = 0.2

# --- Potato on the belt ---
POTATO_START = (-0.6, 0.0, 0.15)
INIT_QUATERNION = (1.0, 1.0, 0.0, 0.0)   # default; overridden by --run
INIT_LINEAR_VELOCITY = (0.0, 0.0, 0.0)
INIT_ANGULAR_VELOCITY = (0.0, 0.0, 0.0)

# --- Object material (locked fixed material) ---
OBJECT_DENSITY = 1000.0
OBJECT_RESTITUTION = 0.4
OBJECT_STATIC_FRICTION = 0.5
OBJECT_DYNAMIC_FRICTION = 0.5

# --- PhysX / capture rates ---
DT = 1.0 / 120.0
MAX_FRAMES = 260                          # long enough to transport + fall off the end
STABILITY_WINDOW = 5
STABILITY_EPSILON = 1e-4

# --- Stereo rig (frames belt + drop zone, baseline across belt width = Y) ---
CAM_BASE = (-0.1, 1.5, 0.6)
CAM_LOOK_AT = (0.0, 0.0, 0.1)
CAM_FOCAL_LENGTH = 24.0
CAM_RESOLUTION = (720, 720)
STEREO_BASELINE = 0.06

SCENARIO = "conveyor"
OUTPUT_SUBDIR = "conveyor"


def _apply_ic():
    global BELT_VELOCITY, INIT_QUATERNION, OUTPUT_SUBDIR, USD_PATH
    if DATASET_MODE:
        BELT_VELOCITY = (_IC_BELT_VELS[VEL_IDX] if MATRIX_MODE
                          else _IC_BELT_VELS[0])
        INIT_QUATERNION = (_IC_POSES[POSE_IDX] if MATRIX_MODE
                            else _IC_POSES[0])
        v_tag = f"v{VEL_IDX}_p{POSE_IDX}" if MATRIX_MODE else "v0_p0"
        OUTPUT_SUBDIR = f"{SCENARIO}/{OBJECT_ID}/{v_tag}"
        USD_PATH = (f"/data2/adrien/clean_dataset/{OBJECT_ID}"
                     f"/{OBJECT_ID}_centered.usd")
    elif MATRIX_MODE:
        BELT_VELOCITY = _IC_BELT_VELS[VEL_IDX]
        INIT_QUATERNION = _IC_POSES[POSE_IDX]
        OUTPUT_SUBDIR = f"conveyor/v{VEL_IDX}_p{POSE_IDX}"


def _add_belt(stage):
    """Create the conveyor belt box and wire it to the conveyor extension.

    Uses a unit cube (size=1) scaled to (length, width, thickness) so final
    extents equal the scale. We do NOT pre-apply RigidBodyAPI: the extension's
    create_conveyor_belt only applies CollisionAPI + PhysxSurfaceVelocityAPI
    when the prim lacks RigidBodyAPI, so we let it own that and set kinematic
    on the resulting API afterward.
    """
    from isaacsim.asset.gen.conveyor import create_conveyor_belt

    belt = UsdGeom.Cube.Define(stage, BELT_PATH)
    belt.CreateSizeAttr(1.0)
    UsdGeom.XformCommonAPI(belt).SetScale((BELT_LENGTH, BELT_WIDTH, BELT_THICKNESS))
    UsdGeom.XformCommonAPI(belt).SetTranslate(BELT_POSITION)

    belt_prim = belt.GetPrim()
    conveyor_node = create_conveyor_belt(stage, belt_prim)
    UsdPhysics.RigidBodyAPI(belt_prim).CreateKinematicEnabledAttr(True)

    conveyor_node.GetAttribute("inputs:direction").Set(Gf.Vec3f(*BELT_DIRECTION))
    graph_path = conveyor_node.GetParent().GetPath()
    stage.GetPrimAtPath(graph_path).GetAttribute("graph:variable:Velocity").Set(BELT_VELOCITY)
    return belt_prim


def run_simulation() -> None:
    _apply_ic()
    _ensure_conveyor_extension()
    stage = setup_stage()

    # Enable kinematic-pairs contact reporting so potato->belt contacts fire.
    set_kinematic_pairs_flags("/PhysicsScene")

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

    belt_prim = _add_belt(stage)
    # Apply physics material to the belt for high grip
    apply_physics_material(
        belt_prim,
        restitution=BELT_RESTITUTION,
        static_friction=BELT_FRICTION,
        dynamic_friction=BELT_FRICTION,
    )

    # --- Potato on the belt ---
    init_quat = Gf.Quatd(
        INIT_QUATERNION[0],
        Gf.Vec3d(INIT_QUATERNION[1], INIT_QUATERNION[2], INIT_QUATERNION[3]),
    )
    potato = load(
        name="Potato", prim_path=USD_PATH, position=POTATO_START,
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

    enable_contact_reporting(["/World/Potato", BELT_PATH], threshold=0.0)

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
