# First run the SimulationApp
import time
import math
from isaacsim import SimulationApp
simulation_app = SimulationApp(launch_config={"headless": False})

# Import Isaac dependencies
import carb
import omni
import carb.settings
import omni.replicator.core as rep
import isaacsim.core.experimental.utils.stage as stage_utils
from pxr import (
    Gf,             # Graphics Foundation: vectors, quaternions, matrices
    PhysxSchema,    # PhysX-specific USD schemas (collision, rigid body, scene)
    UsdPhysics,      # USD physics schemas (collision, rigid body, material)
    Usd,
    UsdGeom,        # USD geometry schemas (Mesh, Cube, Sphere, etc.)
    Sdf,            # USD scene description foundation (for attribute types)
    UsdLux,         # USD lighting schemas (DistantLight, DiskLight, etc.)
)

# PhysX constants
GRAVITY = (0.0, 0.0, -9.81)            # Z-up world, so gravity pulls along -Z
DT = 1.0 / 120.0                        # Capture timestep (s). 120 fps capture rate.
STAGE_TIME_CODES_PER_SECOND = 120       # Timeline rate = capture rate (one frame per update)
PHYSICS_INTERNAL_RATE = 480             # PhysX internal solver rate (Hz). 480/120 = 4 substeps per capture frame.
PHYSX_POSITION_ITERATIONS = 4           # Solver position-correction passes per internal substep
PHYSX_VELOCITY_ITERATIONS = 4           # Solver velocity-correction passes per internal substep

# --- Capture & termination ---
MAX_FRAMES = 100                        # hard cap (300 frames x 1/120 s = 2.5 s)
STABILITY_WINDOW = 5                   # consecutive frames to check for stillness
STABILITY_EPSILON = 1e-4                # m — max positional drift over the window to count as "settled"

# Define dependencies
RESOLUTION = (1280, 720)
CAM_CLIPPING_RANGE = (0.01, 10000.0)
USD_PATH = "/data2/adrien/clean_dataset/2R1-1/2R1-1_centered.usd"

# --- Object material ---
OBJECT_DENSITY = 1000.0                 # kg/m^3 — real potato density (mass = density x volume)
OBJECT_RESTITUTION = 0.35               # bounciness: 0 = dead, 1 = perfectly elastic
OBJECT_STATIC_FRICTION = 0.6            # friction threshold to start sliding
OBJECT_DYNAMIC_FRICTION = 0.6           # friction while sliding
OBJECT_LINEAR_DAMPING = 0.0             # linear air resistance. 0 = none (negligible over 2.5 s)
OBJECT_ANGULAR_DAMPING = 0.0            # rotational air resistance. 0 = none
OBJECT_CONTACT_OFFSET = 0.005           # m — distance at which PhysX begins contact detection

# --- Initial conditions (what the object starts with at t=0) ---
INIT_POSITION = (0.0, 0.0, 0.4)         # (x, y, z) in meters — drop height
INIT_QUATERNION = (1.0, 1.0, 0.0, 0.0)  # (w, x, y, z) — identity = no rotation
INIT_LINEAR_VELOCITY = (0.0, 0.0, 0.0)  # (vx, vy, vz) in m/s — dropped from rest
INIT_ANGULAR_VELOCITY = (0.0, 0.5, 5.0) # (wx, wy, wz) in rad/s — no initial spin

# --- Camera (fixed throughout the trajectory, not tracking) ---
CAM_POS = (0.0, 0.5, 0.25)             # world-space camera position (m)
CAM_LOOK_AT = (0.0, 0.0, 0.10)         # world-space point the camera looks at (m)
CAM_FOCAL_LENGTH = 24.0                # mm
CAM_RESOLUTION = (720, 720)            # (width, height) pixels
CAM_CLIPPING_RANGE = (0.01, 10000.0)   # (near, far) clip planes in meters

RENDER_MODE = "PathTracing"
PATH_TRACING_SPP = 64                   # samples per pixel per frame (64 = clean with denoiser)
ENABLE_DENOISER = True                  # OptiX denoiser cleans up noise from low SPP
RT_SUBFRAMES = 1                        # subframes per capture (in PathTracing, 1 = render SPP samples once)

# ENVIRONMENT VARIABLES:
LIGHT_TYPES = ["SphereLight", "DiskLight", "CylinderLight", "DistantLight", "RectLight"]


def configure_renderer(stage):
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
    stage.SetTimeCodesPerSecond(STAGE_TIME_CODES_PER_SECOND)
    settings.set("/app/player/useFixedTimeStepping", True)
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

def run_simulation() -> None:
    # Set DLSS to Quality mode to reduce low-resolution SDG rendering artifacts.
    print("Setting up the stage")
    stage = stage_utils.create_new_stage(template="gridroom")

    # Check the content of the stage
    """for prim in stage.Traverse():
        path = prim.GetPath().pathString
        prim_type = prim.GetTypeName()
        print(f"  Stage prim: {path} (Type: {prim_type})")
        attributes = prim.GetAttributes()
        for attr in attributes:
            attr_name = attr.GetName()
            attr_value = attr.Get()
            attr_type = attr.GetTypeName()
            print(f"    Attribute: {attr_name} = {attr_value} (Type: {attr_type})")"""    
    configure_renderer(stage)
    # Setup replicator
    rep.orchestrator.set_capture_on_play(False)
    print("[obj1] Setting up camera...")
    rep.functional.create.scope(name="Cameras", parent="/World")
    cam = rep.functional.create.camera(
        parent="/World/Cameras",
        name="Camera",
        position=CAM_POS,
        look_at=CAM_LOOK_AT,
        focal_length=CAM_FOCAL_LENGTH,
        clipping_range=CAM_CLIPPING_RANGE
    )
    rp = rep.create.render_product(cam, CAM_RESOLUTION, name="MainRenderProduct")

    # Define BasicWriter
    backend = rep.backends.get("DiskBackend")
    out_dir = "/home/adrien/Isaacsim_playground/samples/simulations/simple_simulation"
    backend.initialize(output_dir=out_dir)
    print(f"[SimpleSim] Output directory: {out_dir}")
    writer = rep.writers.get("BasicWriter")
    writer.initialize(
        backend=backend,
        rgb=True,
        )
    writer.attach(rp)

    # Create the physics scene and configure PhysX solver iterations for better stability
    rep.functional.physics.create_physics_scene(
        "/PhysicsScene", timeStepsPerSecond=PHYSICS_INTERNAL_RATE
    )
    physx_scene_api = PhysxSchema.PhysxSceneAPI.Apply(
        stage.GetPrimAtPath("/PhysicsScene")
    )
    physx_scene_api.CreateMaxPositionIterationCountAttr(PHYSX_POSITION_ITERATIONS)
    physx_scene_api.CreateMaxVelocityIterationCountAttr(PHYSX_VELOCITY_ITERATIONS)



    # Instantiate a simple potato
    init_quat = Gf.Quatd(
        INIT_QUATERNION[0],
        Gf.Vec3d(INIT_QUATERNION[1], INIT_QUATERNION[2], INIT_QUATERNION[3]),
    )
    print("[SimpleSim] Loading potato asset...")
    potato_prim = rep.functional.create.reference(
        usd_path=USD_PATH,
        parent="/World",
        name="Potato",
        position=(0, 0, 0.8),
        rotation=init_quat
    )
    rep.functional.modify.semantics(potato_prim, {"class": "potato"}, mode="add")
    rep.functional.physics.apply_rigid_body(potato_prim, disableGravity=False)


    # Set density so PhysX computes mass = density x collider_volume
    mass_api = UsdPhysics.MassAPI.Apply(potato_prim)
    mass_api.CreateDensityAttr(OBJECT_DENSITY)

    # Add colliders to all mesh children (3-layer pattern, see add_colliders)
    add_colliders(potato_prim, contact_offset=OBJECT_CONTACT_OFFSET)

    # Bind physics material (restitution + friction)
    apply_physics_material(
        potato_prim,
        restitution=OBJECT_RESTITUTION,
        static_friction=OBJECT_STATIC_FRICTION,
        dynamic_friction=OBJECT_DYNAMIC_FRICTION,
    )

    linear_vel = Gf.Vec3f(*INIT_LINEAR_VELOCITY)
    angular_vel = Gf.Vec3f(*INIT_ANGULAR_VELOCITY)

    # Set the attributes on the potato prim
    potato_prim.GetAttribute("physics:velocity").Set(linear_vel)
    potato_prim.GetAttribute("physics:angularVelocity").Set(angular_vel)

    # Acquisition loop: capture frames until the object settles (or max frames reached)
    timeline = omni.timeline.get_timeline_interface()
    timeline.set_looping(False)
    if not timeline.is_playing():
        timeline.play()

    # Warmup: let the scene settle (physics plugin initializes, collisions register)
    for _ in range(5):
        simulation_app.update()

    print(f"\n  Starting capture loop ({MAX_FRAMES} max frames, "
          f"stability: {STABILITY_WINDOW} frames @ {STABILITY_EPSILON}m)...")
    print(f"  Timeline rate: {STAGE_TIME_CODES_PER_SECOND} fps, "
          f"Physics rate: {PHYSICS_INTERNAL_RATE} Hz "
          f"({PHYSICS_INTERNAL_RATE // STAGE_TIME_CODES_PER_SECOND} substeps/frame)")

    pos_window = []          # last N positions for stability check
    settled = False
    wall_start = time.perf_counter()

    for frame_idx in range(MAX_FRAMES):
        # --- CAPTURE the current frame (do NOT advance timeline) ---
        # delta_time=0.0 = capture at current sim time without advancing.
        # pause_timeline=False = keep timeline playing so the next update can advance.
        rep.orchestrator.step(
            delta_time=0.0,
            rt_subframes=RT_SUBFRAMES,
            pause_timeline=False,
        )

        # --- CHECK STABILITY (before advancing, using current state) ---
        pos, quat, lv, av = read_rigid_body_state(potato_prim)
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
        if settled:
            break

    # Wait for the data to be written to disk and clean up resources
    rep.orchestrator.wait_until_complete()
    writer.detach()
    rp.destroy()

run_simulation()
simulation_app.close()