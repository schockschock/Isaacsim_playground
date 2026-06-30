"""conveyor — potato transported by a conveyor belt then falling off the end.

Uses the isaacsim.asset.gen.conveyor extension's create_conveyor_belt() to
drive a kinematic belt prim via PhysxSurfaceVelocityAPI. The extension
auto-applies RigidBodyAPI/CollisionAPI/PhysxSurfaceVelocityAPI/MeshCollisionAPI
to the belt, so we do NOT re-apply them. The potato is placed on the belt,
transported, then falls off the end into a drop zone. Stereo rig frames the
belt + drop zone.

Run:  conda deactivate && $ISAAC_SIM/python.sh samples/simulations/conveyor.py
"""
import os

from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": True})

import omni.kit.app
import omni.replicator.core as rep
import omni.usd
from pxr import Gf, UsdGeom, UsdPhysics

from common.app import shutdown
from common.world import setup_stage
from common.asset import load, add_colliders, apply_physics_material
from common.replicator import setup_stereo_rig
from common.loop import run_capture_loop, teardown
from common.video import pngs_to_mp4, stereo_pair_to_mp4


def _ensure_conveyor_extension():
    """The conveyor extension is not enabled by default; load it explicitly.
    Must be called before importing isaacsim.asset.gen.conveyor."""
    ext_id = "isaacsim.asset.gen.conveyor"
    mgr = omni.kit.app.get_app().get_extension_manager()
    if not mgr.is_extension_enabled(ext_id):
        mgr.set_extension_enabled_immediate(ext_id, True)

# --- Asset ---
USD_PATH = "/data2/adrien/clean_dataset/2R1-1/2R1-1_centered.usd"

# --- Conveyor belt (a long box, kinematic) ---
BELT_PATH = "/World/ConveyorBelt"
BELT_POSITION = (-0.2, 0.0, 0.05)          # belt top at z=0.1
BELT_LENGTH = 1.1                         # X extent (m)
BELT_WIDTH = 0.4                           # Y extent (m)
BELT_THICKNESS = 0.1                       # Z extent (m)
BELT_VELOCITY = 0.5                        # m/s along +X
BELT_DIRECTION = (1.0, 0.0, 0.0)           # transport toward +X

# --- Potato on the belt ---
POTATO_START = (-0.6, 0.0, 0.15)          # on belt, near -X end
INIT_QUATERNION = (1.0, 1.0, 0.0, 0.0)
INIT_LINEAR_VELOCITY = (0.0, 0.0, 0.0)
INIT_ANGULAR_VELOCITY = (0.0, 0.0, 0.0)

# --- Object material & mass ---
OBJECT_DENSITY = 1000.0
OBJECT_RESTITUTION = 0.2
OBJECT_STATIC_FRICTION = 0.8              # high friction so the belt grips it
OBJECT_DYNAMIC_FRICTION = 0.8

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

OUTPUT_SUBDIR = "conveyor"


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
    # Let the extension apply RigidBodyAPI + CollisionAPI + PhysxSurfaceVelocityAPI.
    conveyor_node = create_conveyor_belt(stage, belt_prim)
    # Now make the belt kinematic (driven by surface velocity, not gravity).
    UsdPhysics.RigidBodyAPI(belt_prim).CreateKinematicEnabledAttr(True)

    # Direction on the conveyor OG node.
    conveyor_node.GetAttribute("inputs:direction").Set(Gf.Vec3f(*BELT_DIRECTION))
    # Velocity lives on the action graph prim as a graph variable.
    graph_path = conveyor_node.GetParent().GetPath()
    stage.GetPrimAtPath(graph_path).GetAttribute("graph:variable:Velocity").Set(BELT_VELOCITY)
    return belt_prim


def run_simulation() -> None:
    _ensure_conveyor_extension()
    stage = setup_stage()

    left_cam, right_cam, rps, writer = setup_stereo_rig(
        base_position=CAM_BASE,
        look_at=CAM_LOOK_AT,
        resolution=CAM_RESOLUTION,
        baseline=STEREO_BASELINE,
        focal_length=CAM_FOCAL_LENGTH,
        output_subdir=OUTPUT_SUBDIR,
    )

    _add_belt(stage)

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
    potato.GetAttribute("physics:velocity").Set(Gf.Vec3f(*INIT_LINEAR_VELOCITY))
    potato.GetAttribute("physics:angularVelocity").Set(Gf.Vec3f(*INIT_ANGULAR_VELOCITY))

    # --- Capture ---
    frames, settled = run_capture_loop(
        simulation_app, potato,
        max_frames=MAX_FRAMES, dt=DT,
        stability_window=STABILITY_WINDOW,
        stability_epsilon=STABILITY_EPSILON,
    )

    teardown(writer, rps)

    # --- MP4 ---
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
