"""object_throw — potato launched sideways, rebounds off a wall then the floor.

The potato is thrown from one side with a tunable initial linear + angular
velocity, rebounds off a vertical wall, then off the ground. A static wall
prim (box collider, no rigid body = immovable) sits across the path. A
wide stereo rig frames the full trajectory (wall + floor impact zones).
All knobs are top-of-file constants.

Run:  conda deactivate && $ISAAC_SIM/python.sh samples/simulations/object_throw.py
"""
import os

from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": True})

import omni.replicator.core as rep
from pxr import Gf, UsdGeom, UsdPhysics

from common.app import shutdown
from common.world import setup_stage
from common.asset import load, add_colliders, apply_physics_material, set_initial_state
from common.replicator import setup_stereo_rig
from common.loop import run_capture_loop, teardown
from common.video import pngs_to_mp4, stereo_pair_to_mp4

# --- Asset ---
USD_PATH = "/data1/adrien/clean_dataset/2R1-1/2R1-1_centered.usd"

# --- Initial conditions (tunable) ---
INIT_POSITION = (0.0, 0.0, 0.4)          # launch from -X side
INIT_QUATERNION = (1.0, 1.0, 0.0, 0.0)
INIT_LINEAR_VELOCITY = (10.0, 0.0, 1.0)    # m/s — toward +X and slightly up
INIT_ANGULAR_VELOCITY = (0.0, 0.0, 8.0)   # rad/s (converted to deg/s for PhysX)

# --- Wall (static collider across the +X side of the path) ---
WALL_PATH = "/World/Wall"
WALL_POSITION = (0.8, 0.0, 0.5)           # center; height 1.0 -> spans z=0..1.0
WALL_THICKNESS = 0.2                       # X extent (m)
WALL_HEIGHT = 1.0                          # Z extent (m)
WALL_WIDTH = 2.0                           # Y extent (m)

# --- Object material & mass ---
OBJECT_DENSITY = 1000.0
OBJECT_RESTITUTION = 0.58                  # bouncier so the rebound is visible
OBJECT_STATIC_FRICTION = 0.6
OBJECT_DYNAMIC_FRICTION = 0.6

# --- PhysX / capture rates ---
DT = 1.0 / 120.0
MAX_FRAMES = 150                          # longer — trajectory takes time
STABILITY_WINDOW = 5
STABILITY_EPSILON = 1e-4

# --- Stereo rig (wide framing, baseline perpendicular to trajectory plane) ---
# Trajectory is in the X-Z plane (Y~0). Camera off +Y, baseline along X.
CAM_BASE = (0.1, 2.5, 0.6)
CAM_LOOK_AT = (0.1, 0.0, 0.3)
CAM_FOCAL_LENGTH = 24.0
CAM_RESOLUTION = (720, 720)
STEREO_BASELINE = 0.06

OUTPUT_SUBDIR = "object_throw"


def _add_wall(stage):
    """Add a static vertical wall: a box with CollisionAPI but no RigidBodyAPI,
    so PhysX treats it as an immovable collider.

    UsdGeom.Cube final extents = size * scale, so use a unit cube (size=1.0)
    and scale by the full (thickness, width, height) in meters.
    """
    wall = UsdGeom.Cube.Define(stage, WALL_PATH)
    wall.CreateSizeAttr(1.0)
    UsdGeom.XformCommonAPI(wall).SetScale((WALL_THICKNESS, WALL_WIDTH, WALL_HEIGHT))
    UsdGeom.XformCommonAPI(wall).SetTranslate(WALL_POSITION)
    UsdPhysics.CollisionAPI.Apply(wall.GetPrim())
    apply_physics_material(
        wall.GetPrim(),
        restitution=OBJECT_RESTITUTION,
        static_friction=OBJECT_STATIC_FRICTION,
        dynamic_friction=OBJECT_DYNAMIC_FRICTION,
    )
    return wall.GetPrim()


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
