#!/usr/bin/env python3
"""
IsaacSim test simulation — single R4-4 free-fall + bounce trajectory.

Purpose
-------
A minimal, heavily-commented IsaacSim script that drops a single potato object
(R4-4) from a fixed height and records its trajectory until it settles.

Designed as a LEARNING TOOL:
  - Every physical parameter lives in the CONSTANTS block (Section 4).
  - Each IsaacSim-specific API call has an explanatory comment.
  - The main() function is linear: setup -> capture -> post-process -> done.
  - No multi-object loops, no retry logic, no resume checks.

Outputs
-------
  {output_dir}/trajectory_0000/
  ├── states.csv                        # 13 cols, no header
  ├── processed/frame_XXXXXX.png        # 720x720 RGB (alpha-composited against black)
  ├── downsampled/frame_XXXXXX_224.png  # 224x224 RGB (Lanczos from 720)
  ├── _raw/                             # raw BasicWriter output (RGBA, for debugging)
  └── trajectory.mp4                    # 720x720 MP4 at 90 fps playback (capture was 120 fps)

Usage
-----
  $ISAAC_PATH/python.sh scripts/test_simulation.py
  $ISAAC_PATH/python.sh scripts/test_simulation.py --output /tmp/test_run
  $ISAAC_PATH/python.sh scripts/test_simulation.py --seed 123

Prerequisites
-------------
  - IsaacSim 6.0.2+ installed, ISAAC_PATH env var set
  - ffmpeg on PATH (for MP4 generation; otherwise MP4 is skipped)
"""

# ============================================================================
# Section 1: Stdlib imports + CLI parsing
# ============================================================================
# IsaacSim requires SimulationApp() to be created BEFORE any omni.* or pxr.*
# import. But we need CLI args first (to configure the launch). So: parse CLI
# with stdlib only, then init SimulationApp, then import IsaacSim modules.

import argparse
import csv
import glob
import math
import os
import random
import shutil
import subprocess
import time
parser = argparse.ArgumentParser(
    description="IsaacSim single-trajectory test simulation (R4-4 free-fall + bounce)"
)
parser.add_argument(
    "--output", type=str,
    default="./test_simulation/",
    help="Output directory (default: ./test_simulation/)",
)
parser.add_argument(
    "--seed", type=int, default=42,
    help="Random seed for reproducibility (default: 42)",
)
args = parser.parse_args()

# ============================================================================
# Section 2: SimulationApp init — the Omniverse runtime bootstrap
# ============================================================================
# SimulationApp starts the Kit application, loads GPU drivers, and sets up the
# USD stage + physics plugin. It MUST be the first IsaacSim call.
# Set HEADLESS = False to run with a GUI window (requires a display).
HEADLESS = True  # Change to False for GUI mode (debugging)

from isaacsim import SimulationApp  # noqa: E402

ISAAC_PATH = os.environ.get(
    "ISAAC_PATH", "/home/adrien/isaacsim/"
)
KIT_PATH = os.path.join(ISAAC_PATH, "apps", "isaacsim.exp.base.python.kit")

# Starting the SimulationApp
launch_cfg = {"headless": HEADLESS}
if os.path.isfile(KIT_PATH):
    launch_cfg["experience"] = KIT_PATH
    print(f"[init] Using kit: {KIT_PATH}")
else:
    print(f"[init] WARNING: kit not found at {KIT_PATH}, using SimulationApp default")

simulation_app = SimulationApp(launch_config=launch_cfg)
print("[init] SimulationApp started")

# ============================================================================
# Section 3: IsaacSim imports — only available AFTER SimulationApp
# ===============================================================rep.=============
# These modules are loaded by the Kit plugin system at startup. Importing them
# before SimulationApp would fail (ModuleNotFoundError).

import numpy as np
import carb.settings
import omni.replicator.core as rep
import omni.timeline
import omni.usd
from PIL import Image
from pxr import (
    Gf,             # Graphics Foundation: vectors, quaternions, matrices
    PhysxSchema,    # PhysX-specific USD schemas (collision, rigid body, scene)
    Sdf,            # Scene Description Foundation: value type names for shaders
    Usd,            # Core USD: Prim, PrimRange, TimeCode
    UsdGeom,        # USD Geometry: Cube, Mesh, XformCommonAPI, Xformable
    UsdPhysics,     # USD Physics: CollisionAPI, RigidBodyAPI, MassAPI, MaterialAPI
    UsdShade,       # USD Shading: Material, Shader, MaterialBindingAPI (for PBR)
)
from isaacsim.storage.native import get_assets_root_path

# ============================================================================
# Section 4: CONSTANTS — edit these to tune the simulation
# ============================================================================

# --- Physics solver ---
# IsaacSim has THREE independent clocks :
#
#   1. TIMELINE (loop dt): controlled by stage timeCodesPerSecond + useFixedTimeStepping.
#      Each simulation_app.update() advances the timeline by 1/timeCodesPerSecond.
#   2. PHYSICS (physics dt): controlled by PhysicsScene.timeStepsPerSecond.
#      Runs N substeps per timeline frame (N = timeStepsPerSecond / timeCodesPerSecond).
#   3. RENDERER: reads /ExternalSimulationTime (written by physics each step) to
#      decide when to render. If physics doesn't step, the renderer doesn't render.
#
# For synchronized physics + rendering:
#   - timeCodesPerSecond = capture rate (120) → one timeline frame per capture
#   - physics timeStepsPerSecond = 480 → 4 physics substeps per timeline frame
#   - useFixedTimeStepping = True → each update advances by exactly 1/120s
#   - Timeline plays CONTINUOUSLY (never pause) so physics steps every update
GRAVITY = (0.0, 0.0, -9.81)            # Z-up world, so gravity pulls along -Z
DT = 1.0 / 120.0                        # Capture timestep (s). 120 fps capture rate.
STAGE_TIME_CODES_PER_SECOND = 120       # Timeline rate = capture rate (one frame per update)
PHYSICS_INTERNAL_RATE = 480             # PhysX internal solver rate (Hz). 480/120 = 4 substeps per capture frame.
PHYSX_POSITION_ITERATIONS = 4           # Solver position-correction passes per internal substep
PHYSX_VELOCITY_ITERATIONS = 1           # Solver velocity-correction passes per internal substep

# --- Object material ---
OBJECT_DENSITY = 1000.0                 # kg/m^3 — real potato density (mass = density x volume)
OBJECT_RESTITUTION = 0.35               # bounciness: 0 = dead, 1 = perfectly elastic
OBJECT_STATIC_FRICTION = 0.6            # friction threshold to start sliding
OBJECT_DYNAMIC_FRICTION = 0.6           # friction while sliding
OBJECT_LINEAR_DAMPING = 0.0             # linear air resistance. 0 = none (negligible over 2.5 s)
OBJECT_ANGULAR_DAMPING = 0.0            # rotational air resistance. 0 = none
OBJECT_CONTACT_OFFSET = 0.005           # m — distance at which PhysX begins contact detection

# --- Ground material ---
GROUND_RESTITUTION = 0.3
GROUND_STATIC_FRICTION = 0.5
GROUND_DYNAMIC_FRICTION = 0.5
GROUND_THICKNESS = 0.3                  # m — visible cube height
GROUND_HALF_EXTENT = 10.0               # m — half-width (ground is 20x20 m)

# --- Initial conditions (what the object starts with at t=0) ---
INIT_POSITION = (0.0, 0.0, 0.8)         # (x, y, z) in meters — drop height
INIT_QUATERNION = (1.0, 0.0, 0.0, 0.0)  # (w, x, y, z) — identity = no rotation
INIT_LINEAR_VELOCITY = (0.0, 0.0, 0.0)  # (vx, vy, vz) in m/s — dropped from rest
INIT_ANGULAR_VELOCITY = (0.0, 0.0, 0.0) # (wx, wy, wz) in rad/s — no initial spin

# --- Camera (fixed throughout the trajectory, not tracking) ---
CAM_POS = (0.0, 0.5, 0.25)             # world-space camera position (m)
CAM_LOOK_AT = (0.0, 0.0, 0.10)         # world-space point the camera looks at (m)
CAM_FOCAL_LENGTH = 24.0                # mm
CAM_RESOLUTION = (720, 720)            # (width, height) pixels
CAM_CLIPPING_RANGE = (0.01, 10000.0)   # (near, far) clip planes in meters

# --- Lighting ---
# Two lighting modes:
#   "directional" — 3 directional lights at different angles + dome fill.
#                   Produces shadows and depth cues. Good for realism.
#   "white_room"  — Dome light only, uniform white, shadows disabled.
#                   Flat, even illumination with NO shadows. Good for clean
#                   dataset capture where shadows would be a confound.
LIGHTING_MODE = "directional"

# Directional mode settings (used when LIGHTING_MODE == "directional")
DISTANT_LIGHT_INTENSITY = 180.0        # directional lights (3 placed at different angles)
DOME_LIGHT_INTENSITY = 800.0           # ambient dome light (sky)
DISTANT_LIGHT_ROTATIONS = [(0, 60, 0), (0, 0, 0), (0, -60, 0)]

# White-room mode settings (used when LIGHTING_MODE == "white_room")
WHITE_ROOM_DOME_INTENSITY = 1000.0     # high intensity for bright, flat illumination
WHITE_ROOM_DOME_COLOR = (1.0, 1.0, 1.0)  # pure white light

# --- Ground plane visual material ---
# White for "white room" look; grey for directional mode.
GROUND_COLOR = (0.5, 0.5, 0.5)         # RGB in [0, 1] — grey for directional lighting
GROUND_ROUGHNESS = 0.5                 # 0 = mirror, 1 = fully matte
GROUND_METALLIC = 0.0                  # 0 = dielectric (non-metal)

# --- Capture & termination ---
MAX_FRAMES = 300                        # hard cap (300 frames x 1/120 s = 2.5 s)
STABILITY_WINDOW = 10                   # consecutive frames to check for stillness
STABILITY_EPSILON = 1e-4                # m — max positional drift over the window to count as "settled"

# --- Renderer ---
# Two modes are available in IsaacSim:
#   "RealTime"   — uses temporal anti-aliasing (TAA) + DLSS. Fast but accumulates
#                  samples across frames using motion vectors. On fast-moving objects
#                  (free-fall, bounce) this produces ghosting/shadow trails because
#                  the motion vectors can't keep up. NOT recommended for datasets.
#   "PathTracing" — renders each frame independently (no temporal accumulation).
#                   No ghosting. Quality controlled by SPP (samples per pixel).
#                   Slower but produces clean, artifact-free images. RECOMMENDED.
RENDER_MODE = "PathTracing"
PATH_TRACING_SPP = 64                   # samples per pixel per frame (64 = clean with denoiser)
ENABLE_DENOISER = True                  # OptiX denoiser cleans up noise from low SPP
RT_SUBFRAMES = 1                        # subframes per capture (in PathTracing, 1 = render SPP samples once)

# --- Output ---
MP4_FPS = 90                            # MP4 playback framerate (capture is 120 fps, so 90 fps = slow-mo)
RESOLUTION_DOWNSAMPLED = (224, 224)     # training-resolution downsample (Lanczos)

# --- Object (the USD file to load) ---
OBJECT_NAME = "3R3-5"
OBJECT_USD_PATH = "/data1/adrien/clean_dataset/3R3-5/3R3-5_centered.usd"


# ============================================================================
# Section 5: Helper functions
# ============================================================================
# These wrap verbose IsaacSim/USD patterns. Each has a docstring explaining WHY.


def configure_renderer():
    """Configure the RTX renderer BEFORE creating any render product.

    Render settings MUST be set before rep.create.render_product() is called,
    because the render product captures the current renderer configuration.

    Why PathTracing instead of RealTime:
        RealTime mode uses temporal anti-aliasing (TAA) which accumulates samples
        across frames using motion vectors. On fast-moving objects (free-fall +
        bounce at 120 fps), the motion vectors can't track the object accurately,
        producing ghost/shadow trails that look like optical-flow artifacts.

        PathTracing renders each frame independently — no temporal accumulation,
        no ghosting. Quality is controlled by SPP (samples per pixel) + denoiser.

    Settings applied:
        /rtx/rendermode                       — "PathTracing" or "RealTime"
        /rtx/pathtracing/spp                  — samples per pixel per frame
        /rtx/pathtracing/totalSpp             — target total samples
        /rtx/pathtracing/optixDenoiser/enabled — AI denoiser for low-SPP cleanup
    """
    settings = carb.settings.get_settings()

    settings.set("/rtx/rendermode", RENDER_MODE)

    if RENDER_MODE == "PathTracing":
        settings.set("/rtx/pathtracing/spp", PATH_TRACING_SPP)
        settings.set("/rtx/pathtracing/totalSpp", PATH_TRACING_SPP)
        settings.set("/rtx/pathtracing/optixDenoiser/enabled", ENABLE_DENOISER)
        print(f"  Renderer: PathTracing (SPP={PATH_TRACING_SPP}, "
              f"denoiser={ENABLE_DENOISER})")
    else:
        # RealTime: disable DLSS frame generation and set quality mode
        settings.set("/rtx-transient/dlssg/enabled", False)
        settings.set("/rtx/post/dlss/execMode", 2)  # 2 = Quality
        print("  Renderer: RealTime (DLSS Quality, no frame gen)")


def randomize_object(custom_object, rng) -> None:
    """Randomize the object's starting rotation and slight X/Y drop position."""
    
    # Randomize X and Y positions slightly so it doesn't fall in the exact center every time
    # The values +/- 0.3 meters keep it safely within the camera's FOV
    x_pos = float(rng.generator.uniform(-0.3, 0.3))
    y_pos = float(rng.generator.uniform(-0.3, 0.3))
    z_pos = 2.5  # Keep the drop height consistent
    
    # Randomize rotation for Roll, Pitch, and Yaw (0 to 360 degrees)
    rot_x = float(rng.generator.uniform(0.0, 360.0))
    rot_y = float(rng.generator.uniform(0.0, 360.0))
    rot_z = float(rng.generator.uniform(0.0, 360.0))
    
    # Apply the new pose
    rep.functional.modify.pose(
        custom_object,
        position_value=(x_pos, y_pos, z_pos),
        rotation_value=(rot_x, rot_y, rot_z)
    )


def look_at_quaternion(cam_pos, cam_look, world_up=(0, 0, 1)):
    """Compute a quaternion that orients a USD camera to face a target point.

    USD cameras look along their local -Z axis by default. This function builds
    a rotation matrix whose columns are [right, up, -forward] and converts it
    to a quaternion using the standard matrix-to-quaternion algorithm.

    Args:
        cam_pos: Camera position in world space (3-tuple).
        cam_look: Point the camera should look at (3-tuple).
        world_up: World up direction (default: Z-up (0, 0, 1)).

    Returns:
        Gf.Quatd: Orientation quaternion (w, x, y, z).
    """
    direction = np.array(cam_look, dtype=float) - np.array(cam_pos, dtype=float)
    direction = direction / np.linalg.norm(direction)
    up = np.array(world_up, dtype=float)
    # Guard against the camera looking straight up or down (direction parallel to up)
    if abs(np.dot(direction, up)) > 0.999:
        up = np.array([0.0, 1.0, 0.0])
    cam_right = np.cross(direction, up)
    cam_right = cam_right / np.linalg.norm(cam_right)
    cam_up = np.cross(cam_right, direction)
    # Build rotation matrix: columns are [right, up, -forward]
    R = np.column_stack([cam_right, cam_up, -direction])
    # Convert rotation matrix to quaternion (Shepperd's method)
    trace = np.trace(R)
    if trace > 0:
        s = math.sqrt(trace + 1.0) * 2.0
        qw = 0.25 * s
        qx = (R[2, 1] - R[1, 2]) / s
        qy = (R[0, 2] - R[2, 0]) / s
        qz = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
        qw = (R[2, 1] - R[1, 2]) / s
        qx = 0.25 * s
        qy = (R[0, 1] + R[1, 0]) / s
        qz = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
        qw = (R[0, 2] - R[2, 0]) / s
        qx = (R[0, 1] + R[1, 0]) / s
        qy = 0.25 * s
        qz = (R[1, 2] + R[2, 1]) / s
    else:
        s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
        qw = (R[1, 0] - R[0, 1]) / s
        qx = (R[0, 2] + R[2, 0]) / s
        qy = (R[1, 2] + R[2, 1]) / s
        qz = 0.25 * s
    return Gf.Quatd(qw, Gf.Vec3d(qx, qy, qz))


def set_fixed_camera_look_at(cam_prim, cam_pos, cam_look):
    """Orient a USD camera prim to look at a target point.

    Sets the xformOp:orient attribute to the look-at quaternion. The orient
    op must already exist on the prim (add it via xform.AddOrientOp() if not).

    Args:
        cam_prim: USD camera prim (from rep.functional.create.camera).
        cam_pos: Camera position in world space.
        cam_look: Target point to look at.
    """
    target_quat = look_at_quaternion(cam_pos, cam_look)
    orient_attr = cam_prim.GetAttribute("xformOp:orient")
    if orient_attr:
        # Match the attribute's native type (Quatf or Quatd)
        type_name = str(orient_attr.GetTypeName())
        if "Quatf" in type_name or "quatf" in type_name.lower():
            quat_f = Gf.Quatf(
                float(target_quat.GetReal()),
                Gf.Vec3f(
                    float(target_quat.GetImaginary()[0]),
                    float(target_quat.GetImaginary()[1]),
                    float(target_quat.GetImaginary()[2]),
                ),
            )
            orient_attr.Set(quat_f)
        else:
            orient_attr.Set(target_quat)


def add_colliders(root_prim, contact_offset=0.005, rest_offset=0.0):
    """Apply collision geometry to all Mesh descendants of root_prim.

    IsaacSim requires THREE collision APIs on each mesh for proper PhysX contact:
      1. UsdPhysics.CollisionAPI    — enables collision for this prim
      2. PhysxSchema.PhysxCollisionAPI — sets PhysX-specific contact/rest offsets
      3. UsdPhysics.MeshCollisionAPI   — tells PhysX to use convex hull approximation

    Without all three, the object may fall through the ground or not collide.

    Args:
        root_prim: The root prim to scan for mesh children.
        contact_offset: Distance (m) at which PhysX starts contact detection.
        rest_offset: Penetration depth (m) below which contacts are suppressed.
    """
    for desc_prim in Usd.PrimRange(root_prim):
        if not (desc_prim.IsA(UsdGeom.Mesh) or desc_prim.IsA(UsdGeom.Gprim)):
            continue

        # 1. Enable collision on this prim
        if not desc_prim.HasAPI(UsdPhysics.CollisionAPI):
            collision_api = UsdPhysics.CollisionAPI.Apply(desc_prim)
        else:
            collision_api = UsdPhysics.CollisionAPI(desc_prim)
        collision_api.CreateCollisionEnabledAttr(True)

        # 2. Set PhysX contact/rest offsets
        if not desc_prim.HasAPI(PhysxSchema.PhysxCollisionAPI):
            physx_collision_api = PhysxSchema.PhysxCollisionAPI.Apply(desc_prim)
        else:
            physx_collision_api = PhysxSchema.PhysxCollisionAPI(desc_prim)
        physx_collision_api.CreateContactOffsetAttr(contact_offset)
        physx_collision_api.CreateRestOffsetAttr(rest_offset)

        # 3. Use convex hull approximation (fast and stable for rigid bodies)
        if desc_prim.IsA(UsdGeom.Mesh):
            if not desc_prim.HasAPI(UsdPhysics.MeshCollisionAPI):
                mesh_collision_api = UsdPhysics.MeshCollisionAPI.Apply(desc_prim)
            else:
                mesh_collision_api = UsdPhysics.MeshCollisionAPI(desc_prim)
            mesh_collision_api.CreateApproximationAttr().Set("convexHull")


def apply_physics_material(prim, restitution, static_friction, dynamic_friction):
    """Create a physics Material prim and bind it to a collision prim.

    Physics materials control restitution (bounciness) and friction. They are
    bound via the "material:binding:physics" relationship — NOT "material:binding"
    (which is for rendering shaders). The combine mode is set to "maximum" so
    the higher coefficient between two colliding bodies wins.

    Args:
        prim: The collision prim to bind the material to.
        restitution: Coefficient of restitution (0 = inelastic, 1 = elastic).
        static_friction: Static friction coefficient.
        dynamic_friction: Dynamic (kinetic) friction coefficient.
    """
    stage = omni.usd.get_context().get_stage()
    material_path = f"/World/PhysicsMaterial_{prim.GetName()}"

    material_prim = stage.DefinePrim(material_path, "Material")
    material_api = UsdPhysics.MaterialAPI.Apply(material_prim)
    material_api.CreateRestitutionAttr().Set(restitution)
    material_api.CreateStaticFrictionAttr().Set(static_friction)
    material_api.CreateDynamicFrictionAttr().Set(dynamic_friction)

    # PhysX combine modes: "maximum" = use the higher of the two colliding materials
    try:
        physx_mat = PhysxSchema.PhysxMaterialAPI.Apply(material_prim)
        physx_mat.CreateRestitutionCombineModeAttr().Set("maximum")
        physx_mat.CreateFrictionCombineModeAttr().Set("maximum")
    except Exception:
        pass  # Not critical — defaults work

    # Bind the material to the prim using the physics binding namespace
    prim.CreateRelationship("material:binding:physics", False).AddTarget(
        material_prim.GetPath()
    )


def read_rigid_body_state(prim):
    """Read position, orientation, and velocities from a USD rigid body prim.

    PhysX writes the current state to USD attributes every physics step. We
    read these directly — no finite differences. This gives us the solver's
    internal values, which is what the dataset spec requires.

    Args:
        prim: The rigid body USD prim.

    Returns:
        Tuple of (position, quaternion, linear_velocity, angular_velocity).
        position: Gf.Vec3d (x, y, z) in meters
        quaternion: Gf.Quatf or Gf.Quatd (w, x, y, z)
        linear_velocity: Gf.Vec3f (vx, vy, vz) in m/s
        angular_velocity: Gf.Vec3f (wx, wy, wz) in rad/s
    """
    pos = prim.GetAttribute("xformOp:translate").Get()
    quat = prim.GetAttribute("xformOp:orient").Get()
    lv = prim.GetAttribute("physics:velocity").Get()
    av = prim.GetAttribute("physics:angularVelocity").Get()
    return pos, quat, lv, av


def alpha_composite_against_black(rgba_path, output_path):
    """Composite an RGBA image against black and save as RGB PNG.

    IsaacSim's BasicWriter outputs RGBA. The alpha channel is used to composite
    the render against a black background, producing a clean RGB image.

    Args:
        rgba_path: Path to the source RGBA (or RGB) image.
        output_path: Path to save the composited RGB PNG.
    """
    img = Image.open(rgba_path)
    os.makedirs(os.path.dirname(output_path) or ".", exist_ok=True)
    if img.mode == "RGBA":
        arr = np.array(img, dtype=np.uint8)
        rgb = arr[..., :3].astype(np.float32)
        alpha = arr[..., 3:4].astype(np.float32) / 255.0
        composited = np.round(rgb * alpha).astype(np.uint8)
        Image.fromarray(composited, "RGB").save(output_path)
    elif img.mode == "RGB":
        img.save(output_path)
    else:
        img.convert("RGB").save(output_path)


def downsample_image(src_path, dst_path, size=(224, 224)):
    """Downsample an image to the target size using Lanczos resampling.

    Args:
        src_path: Path to the source image.
        dst_path: Path to save the downsampled image.
        size: Target (width, height) in pixels.
    """
    img = Image.open(src_path)
    img = img.resize(size, Image.LANCZOS)
    os.makedirs(os.path.dirname(dst_path) or ".", exist_ok=True)
    img.save(dst_path)


def make_video(frames_dir, output_path, fps=90):
    """Stitch PNG frames into an MP4 video using ffmpeg.

    Reads all frame_*.png files from frames_dir (sorted by name) and encodes
    them as H.264 MP4 at the given frame rate. Skips silently if ffmpeg is
    not on PATH.

    Args:
        frames_dir: Directory containing frame_XXXXXX.png files.
        output_path: Path for the output MP4 file.
        fps: Playback frame rate (default: 90 — slower than the 120 fps capture).

    Returns:
        True if video was generated, False otherwise.
    """
    if shutil.which("ffmpeg") is None:
        print("  [video] ffmpeg not found on PATH — skipping MP4 generation")
        return False

    cmd = [
        "ffmpeg", "-y",
        "-framerate", str(fps),
        "-pattern_type", "glob",
        "-i", os.path.join(frames_dir, "frame_*.png"),
        "-c:v", "libx264",
        "-pix_fmt", "yuv420p",
        "-loglevel", "error",
        output_path,
    ]
    try:
        subprocess.run(cmd, check=True, timeout=120)
        return True
    except (subprocess.CalledProcessError, subprocess.TimeoutExpired) as e:
        print(f"  [video] ffmpeg failed: {e}")
        return False


def count_bounces(states, z_threshold=0.05):
    """Count ground-contact bounces by detecting vz sign flips near the ground.

    A bounce is counted when vz changes from negative (falling) to positive
    (rising) while z is below z_threshold (near the ground plane).

    Args:
        states: List of (pos, quat, lv, av) tuples.
        z_threshold: Maximum z (m) to count as "near ground".

    Returns:
        Number of bounces.
    """
    n_bounces = 0
    for i in range(1, len(states)):
        vz_prev = float(states[i - 1][2][2])
        vz_curr = float(states[i][2][2])
        z_curr = float(states[i][0][2])
        if vz_prev < 0 and vz_curr >= 0 and z_curr < z_threshold:
            n_bounces += 1
    return n_bounces


# ============================================================================
# Section 6: Main simulation
# ============================================================================

def main():
    """Run a single R4-4 free-fall + bounce trajectory.

    Linear flow:
      1. Seed RNGs for reproducibility
      2. Create USD stage with lights, ground, physics scene
      3. Load R4-4 as a rigid body with colliders + physics material
      4. Set up camera + Replicator render product + BasicWriter
      5. Run capture loop until the object settles or MAX_FRAMES is hit
      6. Post-process: composite RGBA -> RGB, downsample to 224, write CSV
      7. Generate MP4 video
      8. Print physical sanity summary
    """

    # ------------------------------------------------------------------
    # 0. Output directory setup
    # ------------------------------------------------------------------
    output_base = os.path.abspath(args.output)
    traj_dir = os.path.join(output_base, "trajectory_0000")
    raw_dir = os.path.join(traj_dir, "_raw")
    processed_dir = os.path.join(traj_dir, "processed")
    downsampled_dir = os.path.join(traj_dir, "downsampled")

    # Clean any previous run (test script always starts fresh)
    if os.path.isdir(traj_dir):
        shutil.rmtree(traj_dir)
    os.makedirs(raw_dir, exist_ok=True)
    os.makedirs(processed_dir, exist_ok=True)
    os.makedirs(downsampled_dir, exist_ok=True)

    # Seed all RNGs (PhysX, Python random, NumPy, Replicator)
    random.seed(args.seed)
    np.random.seed(args.seed)
    rep.set_global_seed(args.seed)

    print(f"\n{'=' * 60}")
    print(f"IsaacSim Test Simulation — {OBJECT_NAME}")
    print(f"{'=' * 60}")
    print(f"  Output:  {traj_dir}")
    print(f"  Object:  {OBJECT_NAME} ({OBJECT_USD_PATH})")
    print(f"  Seed:    {args.seed}")
    print(f"  dt:      {DT:.5f}s (capture {int(1/DT)} fps, internal {PHYSICS_INTERNAL_RATE} Hz)")
    print(f"  Max frames: {MAX_FRAMES} ({MAX_FRAMES * DT:.2f}s)")
    print(f"  Drop height: {INIT_POSITION[2]}m, restitution: {OBJECT_RESTITUTION}")
    print(f"{'=' * 60}")

    # ------------------------------------------------------------------
    # 1. Create a new USD stage (the "world" container)
    # ------------------------------------------------------------------
    # new_stage() clears any existing stage and creates a fresh one.
    omni.usd.get_context().new_stage()
    stage = omni.usd.get_context().get_stage()

    # Set the timeline rate = capture rate. With useFixedTimeStepping=True,
    # each simulation_app.update() advances the timeline by exactly 1/120s.
    # This is the first of the three clocks (see CONSTANTS block for explanation).
    stage.SetTimeCodesPerSecond(STAGE_TIME_CODES_PER_SECOND)
    carb.settings.get_settings().set("/app/player/useFixedTimeStepping", True)

    # /World is the root Xform for all scene content
    rep.functional.create.xform(name="World")

    # ------------------------------------------------------------------
    # 2. Add lighting
    # ------------------------------------------------------------------
    # Two modes controlled by the LIGHTING_MODE constant:
    #   "directional" — 3 directional lights + dome fill (shadows, depth cues)
    #   "white_room"  — dome light only, uniform white, shadows disabled
    #                   (flat, even illumination — no shadows at all)

    if LIGHTING_MODE == "directional":
        for i, rotation in enumerate(DISTANT_LIGHT_ROTATIONS):
            rep.functional.create.distant_light(
                intensity=DISTANT_LIGHT_INTENSITY,
                rotation=rotation,
                name=f"DistantLight_{i}",
            )
        rep.functional.create.dome_light(
            intensity=DOME_LIGHT_INTENSITY,
            name="DomeLight",
        )
        print("  Lighting: directional (3 distant + dome, with shadows)")
    elif LIGHTING_MODE == "white_room":
        # Dome light only: uniform omnidirectional illumination from all
        # directions. shadow:enable=False guarantees no cast shadows.
        # This produces a flat, shadowless "studio" look.
        dome = rep.functional.create.dome_light(
            intensity=WHITE_ROOM_DOME_INTENSITY,
            color=WHITE_ROOM_DOME_COLOR,
            name="DomeLight",
        )
        # Disable shadows on the dome light for a truly shadowless scene.
        # The attribute doesn't exist by default, so we create it first.
        dome.CreateAttribute("shadow:enable", Sdf.ValueTypeNames.Bool).Set(False)
        print(f"  Lighting: white_room (dome only, shadows disabled, "
              f"intensity={WHITE_ROOM_DOME_INTENSITY})")
    else:
        raise ValueError(f"Unknown LIGHTING_MODE: {LIGHTING_MODE!r} "
                         f"(expected 'directional' or 'white_room')")

    # ------------------------------------------------------------------
    # 3. Create the ground plane (visible Cube with a physics collider)
    # ------------------------------------------------------------------
    # We use a scaled Cube instead of a Plane schema because Plane has no
    # GetSizeAttr in USD. The cube is scaled to 20x20 m and sunk below z=0
    # so its top surface is at z=0.
    ground_path = "/World/GroundPlane"
    ground_prim = stage.DefinePrim(ground_path, "Cube")
    cube = UsdGeom.Cube(ground_prim)
    cube.GetSizeAttr().Set(1.0)
    UsdGeom.XformCommonAPI(ground_prim).SetScale(
        (GROUND_HALF_EXTENT * 2, GROUND_HALF_EXTENT * 2, GROUND_THICKNESS)
    )
    UsdGeom.XformCommonAPI(ground_prim).SetTranslate((0, 0, -GROUND_THICKNESS / 2))

    # Ground collider (same pattern as object: CollisionAPI + PhysxCollisionAPI)
    UsdPhysics.CollisionAPI.Apply(ground_prim)
    physx_collision = PhysxSchema.PhysxCollisionAPI.Apply(ground_prim)
    physx_collision.CreateContactOffsetAttr(OBJECT_CONTACT_OFFSET)
    physx_collision.CreateRestOffsetAttr(0.0)

    # Ground visual material (PBR — color/roughness/metallic from CONSTANTS)
    # White ground completes the "white room" look; grey for directional mode.
    ground_material = UsdShade.Material.Define(stage, "/World/GroundMaterial")
    pbr_shader = UsdShade.Shader.Define(stage, "/World/GroundMaterial/PBRShader")
    pbr_shader.CreateIdAttr("UsdPreviewSurface")
    pbr_shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(
        Gf.Vec3f(*GROUND_COLOR)
    )
    pbr_shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(GROUND_ROUGHNESS)
    pbr_shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(GROUND_METALLIC)
    ground_material.CreateSurfaceOutput().ConnectToSource(
        pbr_shader.ConnectableAPI(), "surface"
    )
    UsdShade.MaterialBindingAPI.Apply(ground_prim).Bind(ground_material)

    # Ground physics material (restitution + friction)
    apply_physics_material(
        ground_prim,
        restitution=GROUND_RESTITUTION,
        static_friction=GROUND_STATIC_FRICTION,
        dynamic_friction=GROUND_DYNAMIC_FRICTION,
    )

    # ------------------------------------------------------------------
    # 4. Create the physics scene (configures the PhysX solver)
    # ------------------------------------------------------------------
    # timeStepsPerSecond=480 means PhysX internally runs at 480 Hz.
    # With DT=1/120, each capture frame advances 4 internal physics substeps
    # (480/120 = 4). Position/velocity iterations control solver accuracy.
    rep.functional.physics.create_physics_scene(
        "/PhysicsScene", timeStepsPerSecond=PHYSICS_INTERNAL_RATE
    )
    physx_scene_api = PhysxSchema.PhysxSceneAPI.Apply(
        stage.GetPrimAtPath("/PhysicsScene")
    )
    physx_scene_api.CreateMaxPositionIterationCountAttr(PHYSX_POSITION_ITERATIONS)
    physx_scene_api.CreateMaxVelocityIterationCountAttr(PHYSX_VELOCITY_ITERATIONS)

    # ------------------------------------------------------------------
    # 5. Load the object (R4-4) as a rigid body with colliders + material
    # ------------------------------------------------------------------
    # Convert INIT_QUATERNION (w, x, y, z) tuple to Gf.Quatd.
    # Gf.Quatd takes (real_part, imaginary_vec3) = (w, (x, y, z)).
    init_quat = Gf.Quatd(
        INIT_QUATERNION[0],
        Gf.Vec3d(INIT_QUATERNION[1], INIT_QUATERNION[2], INIT_QUATERNION[3]),
    )

    # Reference the object USD into our stage under /World
    # (hyphens are not valid in USD prim names, so "R4-4" -> "R4_4")
    r4_prim = rep.functional.create.reference(
        usd_path=OBJECT_USD_PATH,
        parent="/World",
        name=OBJECT_NAME.replace("-", "_"),
        position=INIT_POSITION,
        rotation=init_quat,
    )

    # Make it a rigid body (PhysX will simulate gravity + collisions)
    rep.functional.physics.apply_rigid_body(r4_prim, disableGravity=False)

    # Set density so PhysX computes mass = density x collider_volume
    mass_api = UsdPhysics.MassAPI.Apply(r4_prim)
    mass_api.CreateDensityAttr(OBJECT_DENSITY)

    # Add colliders to all mesh children (3-layer pattern, see add_colliders)
    add_colliders(r4_prim, contact_offset=OBJECT_CONTACT_OFFSET)

    # Bind physics material (restitution + friction)
    apply_physics_material(
        r4_prim,
        restitution=OBJECT_RESTITUTION,
        static_friction=OBJECT_STATIC_FRICTION,
        dynamic_friction=OBJECT_DYNAMIC_FRICTION,
    )

    # Set linear/angular damping (air resistance; 0 = none)
    try:
        physx_rb_api = PhysxSchema.PhysxRigidBodyAPI.Apply(r4_prim)
        physx_rb_api.CreateLinearDampingAttr(OBJECT_LINEAR_DAMPING)
        physx_rb_api.CreateAngularDampingAttr(OBJECT_ANGULAR_DAMPING)
    except AttributeError:
        print("  WARNING: Could not set damping via PhysxRigidBodyAPI")

    # Set initial velocities (if non-zero) — PhysX reads these at sim start
    if any(v != 0.0 for v in INIT_LINEAR_VELOCITY):
        rb_api = UsdPhysics.RigidBodyAPI(r4_prim)
        rb_api.CreateVelocityAttr().Set(Gf.Vec3f(*INIT_LINEAR_VELOCITY))
        print(f"  Initial linear velocity: {INIT_LINEAR_VELOCITY}")
    if any(v != 0.0 for v in INIT_ANGULAR_VELOCITY):
        rb_api = UsdPhysics.RigidBodyAPI(r4_prim)
        rb_api.CreateAngularVelocityAttr().Set(Gf.Vec3f(*INIT_ANGULAR_VELOCITY))
        print(f"  Initial angular velocity: {INIT_ANGULAR_VELOCITY}")

    print(f"  Initial position: {INIT_POSITION}")
    print(f"  Initial quaternion (w,x,y,z): {INIT_QUATERNION}")

    # ------------------------------------------------------------------
    # 6. Set up the camera (fixed position, looking at the drop zone)
    # ------------------------------------------------------------------
    cam_prim = rep.functional.create.camera(
        parent="/World",
        name="CaptureCamera",
        position=CAM_POS,
        focal_length=CAM_FOCAL_LENGTH,
        clipping_range=CAM_CLIPPING_RANGE,
    )

    # Add orient op if it doesn't exist, then set the look-at quaternion
    xform = UsdGeom.Xformable(cam_prim)
    if not cam_prim.GetAttribute("xformOp:orient"):
        xform.AddOrientOp()
    set_fixed_camera_look_at(cam_prim, CAM_POS, CAM_LOOK_AT)
    print(f"  Camera: pos={CAM_POS}, look_at={CAM_LOOK_AT}, "
          f"focal={CAM_FOCAL_LENGTH}mm, res={CAM_RESOLUTION}")

    cam_path = str(cam_prim.GetPath())

    # ------------------------------------------------------------------
    # 7. Set up Replicator render product + BasicWriter
    # ------------------------------------------------------------------
    # IMPORTANT: configure the renderer BEFORE creating the render product.
    # The render product captures the renderer settings at creation time.
    # If we set render mode after, it won't take effect for this product.
    configure_renderer()

    # The render product defines what the camera sees (resolution, anti-aliasing).
    # BasicWriter writes the rendered frames to disk as PNGs.
    rep.orchestrator.set_capture_on_play(False)
    rp = rep.create.render_product(cam_prim, CAM_RESOLUTION, name="trajectory_cam")

    backend = rep.backends.get("DiskBackend")
    backend.initialize(output_dir=raw_dir)
    writer = rep.writers.get("BasicWriter")
    writer.initialize(
        backend=backend,
        rgb=True,
        image_output_format="png",
    )
    writer.attach([rp])

    # ------------------------------------------------------------------
    # 8. Run the simulation + capture loop (SYNCHRONIZED physics + rendering)
    # ------------------------------------------------------------------
    # CRITICAL: This loop follows the official IsaacSim pattern (see
    # object_based_sdg.py) to guarantee 1:1 frame-to-state synchronization.
    #
    # The loop separates CAPTURE from ADVANCEMENT:
    #   1. CAPTURE: rep.orchestrator.step(delta_time=0.0, pause_timeline=False)
    #      - Renders the CURRENT frame (at the current simulation time)
    #      - delta_time=0.0 means "do NOT advance the timeline, just capture"
    #      - pause_timeline=False keeps the timeline playing so physics can step
    #   2. READ STATE: read pos/quat/vel from USD attrs — same sim time as the render
    #   3. ADVANCE: simulation_app.update()
    #      - Advances the timeline by 1/timeCodesPerSecond (1/120s)
    #      - Physics runs 4 substeps (480 Hz / 120 Hz)
    #      - /ExternalSimulationTime is updated — the renderer will see it next frame
    #
    # Why NOT use step(delta_time=DT, pause_timeline=True)?
    #   - pause_timeline=True stops the timeline, preventing physics from stepping
    #     via the normal per-frame ordering.
    #   - The multi-tick renderer reads /ExternalSimulationTime (written by physics).
    #     If physics doesn't step, sim time doesn't advance, and the renderer
    #     SKIPS the capture — causing dropped frames (state rows > rendered frames).
    timeline = omni.timeline.get_timeline_interface()
    timeline.set_looping(False)
    timeline.play()

    # Warmup: let the scene settle (physics plugin initializes, collisions register)
    for _ in range(5):
        simulation_app.update()

    print(f"\n  Starting capture loop ({MAX_FRAMES} max frames, "
          f"stability: {STABILITY_WINDOW} frames @ {STABILITY_EPSILON}m)...")
    print(f"  Timeline rate: {STAGE_TIME_CODES_PER_SECOND} fps, "
          f"Physics rate: {PHYSICS_INTERNAL_RATE} Hz "
          f"({PHYSICS_INTERNAL_RATE // STAGE_TIME_CODES_PER_SECOND} substeps/frame)")

    states = []
    pos_window = []          # last N positions for stability check
    settled = False
    wall_start = time.perf_counter()

    for frame_idx in range(MAX_FRAMES):
        # --- 1. CAPTURE the current frame (do NOT advance timeline) ---
        # delta_time=0.0 = capture at current sim time without advancing.
        # pause_timeline=False = keep timeline playing so the next update can advance.
        rep.orchestrator.step(
            delta_time=0.0,
            rt_subframes=RT_SUBFRAMES,
            pause_timeline=False,
        )

        # --- 2. READ STATE at the same sim time as the render ---
        # The render and this state read happen at the same simulation time,
        # guaranteeing frame[i] and state[i] are perfectly synchronized.
        pos, quat, lv, av = read_rigid_body_state(r4_prim)
        states.append((pos, quat, lv, av))

        # --- 3. CHECK STABILITY (before advancing, using current state) ---
        pos_window.append((float(pos[0]), float(pos[1]), float(pos[2])))
        if len(pos_window) > STABILITY_WINDOW:
            pos_window.pop(0)

        if len(pos_window) == STABILITY_WINDOW:
            xs = [p[0] for p in pos_window]
            ys = [p[1] for p in pos_window]
            zs = [p[2] for p in pos_window]
            if (max(xs) - min(xs) < STABILITY_EPSILON and
                max(ys) - min(ys) < STABILITY_EPSILON and
                max(zs) - min(zs) < STABILITY_EPSILON):
                settled = True
                print(f"  Settled at frame {frame_idx} "
                      f"({frame_idx * DT:.3f}s)")
                break

        # Progress print every 30 frames (~0.25s intervals)
        if frame_idx % 30 == 0:
            speed = math.sqrt(float(lv[0]) ** 2 + float(lv[1]) ** 2 + float(lv[2]) ** 2)
            ang_speed = math.sqrt(float(av[0]) ** 2 + float(av[1]) ** 2 + float(av[2]) ** 2)
            print(f"  frame {frame_idx:3d}/{MAX_FRAMES}  "
                  f"pos=({float(pos[0]):.3f},{float(pos[1]):.3f},{float(pos[2]):.3f})  "
                  f"|v|={speed:.3f}m/s  |w|={ang_speed:.3f}rad/s")

        # --- 4. ADVANCE physics by one timeline frame (1/120s = 4 substeps) ---
        # This is the key: simulation_app.update() advances the timeline
        # (which triggers physics substeps), updates /ExternalSimulationTime,
        # and lets the renderer prepare for the next capture.
        simulation_app.update()

    # Wait for any async writes to complete, then stop the timeline
    rep.orchestrator.wait_until_complete()
    timeline.stop()

    wall_duration = time.perf_counter() - wall_start
    n_captured = len(states)
    print(f"\n  Capture complete: {n_captured} frames in {wall_duration:.1f}s "
          f"({wall_duration / max(n_captured, 1):.3f}s/frame)")

    if not settled:
        print(f"  WARNING: did not settle within {MAX_FRAMES} frames "
              f"({MAX_FRAMES * DT:.2f}s)")

    # ------------------------------------------------------------------
    # 9. Post-process: find raw RGB files, composite, downsample
    # ------------------------------------------------------------------
    # BasicWriter outputs files like "rgb_000000.png" (possibly in a subdirectory).
    # We find them via recursive glob, sort by frame number, then:
    #   - Alpha-composite each RGBA -> 720x720 RGB  (processed/)
    #   - Downsample 720 -> 224x224                  (downsampled/)
    print("  Post-processing images...")

    raw_rgb_files = sorted(
        glob.glob(os.path.join(raw_dir, "**", "rgb_*.png"), recursive=True)
    )
    # Sort by the integer frame number embedded in the filename
    try:
        raw_rgb_files.sort(
            key=lambda p: int(os.path.basename(p)[4:-4])  # "rgb_000042.png" -> 42
        )
    except (ValueError, IndexError):
        pass  # fallback to lexicographic sort

    print(f"  Found {len(raw_rgb_files)} raw RGB frames")

    n_processed = 0
    for i, rgb_path in enumerate(raw_rgb_files[:n_captured]):
        composite_path = os.path.join(processed_dir, f"frame_{i:06d}.png")
        alpha_composite_against_black(rgb_path, composite_path)

        downsample_path = os.path.join(downsampled_dir, f"frame_{i:06d}_224.png")
        downsample_image(composite_path, downsample_path, RESOLUTION_DOWNSAMPLED)
        n_processed += 1

    print(f"  Processed {n_processed} frames (720x720 + 224x224)")

    # ------------------------------------------------------------------
    # 10. Write states.csv — 13 columns, no header
    # ------------------------------------------------------------------
    # Columns: x, y, z, qw, qx, qy, qz, vx, vy, vz, wx, wy, wz
    csv_path = os.path.join(traj_dir, "states.csv")
    with open(csv_path, "w", newline="") as f:
        writer_csv = csv.writer(f)
        for pos, quat, lv, av in states:
            writer_csv.writerow([
                float(pos[0]), float(pos[1]), float(pos[2]),
                float(quat.GetReal()),
                float(quat.GetImaginary()[0]),
                float(quat.GetImaginary()[1]),
                float(quat.GetImaginary()[2]),
                float(lv[0]), float(lv[1]), float(lv[2]),
                float(av[0]), float(av[1]), float(av[2]),
            ])
    print(f"  Wrote {csv_path} ({n_captured} rows)")

    # ------------------------------------------------------------------
    # 11. Generate MP4 video (720x720 at 90 fps playback)
    # ------------------------------------------------------------------
    video_path = os.path.join(traj_dir, "trajectory.mp4")
    print(f"  Generating video: {video_path} ({MP4_FPS} fps)...")
    if make_video(processed_dir, video_path, fps=MP4_FPS):
        print(f"  Video written: {video_path}")

    # ------------------------------------------------------------------
    # 12. Print physical sanity summary
    # ------------------------------------------------------------------
    # This helps validate the simulation is physically correct:
    #   - Bounce count should be >= 1 (object rebounds at least once)
    #   - Max height should be close to INIT_POSITION[2]
    #   - Energy loss ratio should be close to 1.0 (all PE dissipated by the end)

    max_height = max(float(s[0][2]) for s in states) if states else 0.0
    n_bounces = count_bounces(states)
    settling_time = n_captured * DT

    # Read mass (PhysX computes it from density x collider volume)
    mass = 0.0
    try:
        mass_attr = r4_prim.GetAttribute("physics:mass")
        if mass_attr and mass_attr.IsValid():
            v = mass_attr.Get()
            if v is not None:
                mass = float(v)
    except Exception:
        pass
    if mass <= 0.0:
        mass = 0.08  # fallback (typical potato ~80g)
        print(f"  WARNING: could not read physics:mass, using fallback {mass} kg")

    g_mag = abs(GRAVITY[2])
    init_pe = mass * g_mag * INIT_POSITION[2]

    if states:
        final_pos = states[-1][0]
        final_lv = states[-1][2]
        final_ke = 0.5 * mass * (
            float(final_lv[0]) ** 2 + float(final_lv[1]) ** 2 + float(final_lv[2]) ** 2
        )
        final_pe = mass * g_mag * float(final_pos[2])
        final_total_e = final_ke + final_pe
    else:
        final_pos = (0, 0, 0)
        final_total_e = 0.0

    energy_loss = (init_pe - final_total_e) / init_pe if init_pe > 0 else 0.0

    print(f"\n{'=' * 60}")
    print("SIMULATION SUMMARY")
    print(f"{'=' * 60}")
    print(f"  Object:              {OBJECT_NAME}")
    print(f"  Frames captured:     {n_captured} ({settling_time:.3f}s)")
    print(f"  Settled:             {settled}")
    print(f"  Bounces:             {n_bounces}")
    print(f"  Max height:          {max_height:.4f} m")
    print(f"  Final position:      ({float(final_pos[0]):.4f}, "
          f"{float(final_pos[1]):.4f}, {float(final_pos[2]):.4f})")
    print(f"  Mass:                {mass:.4f} kg")
    print(f"  Initial PE:          {init_pe:.4f} J")
    print(f"  Final total energy:  {final_total_e:.4f} J")
    print(f"  Energy loss ratio:   {energy_loss:.4f}")
    print(f"  Output:              {traj_dir}")
    print(f"{'=' * 60}")

    # ------------------------------------------------------------------
    # 13. Clean up
    # ------------------------------------------------------------------
    simulation_app.close()
    print("[done] SimulationApp closed")


if __name__ == "__main__":
    main()
