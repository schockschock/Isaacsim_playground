"""Stage setup: ground plane, dome light (shadows off), physics scene."""
import carb
import omni.replicator.core as rep
import omni.usd
from pxr import Gf, PhysxSchema, Sdf, UsdGeom, UsdPhysics, UsdLux
import isaacsim.core.experimental.utils.stage as stage_utils


DOME_LIGHT_INTENSITY = 1000.0
DOME_LIGHT_COLOR = (1.0, 1.0, 1.0)
DOME_SHADOWS_ENABLED = False
DISTANT_LIGHT_INTENSITY = 180.0
ROTATION = (0, 60, 0)
RENDER_MODE = "PathTracing"
PATH_TRACING_SPP = 64                   # samples per pixel per frame (64 = clean with denoiser)
ENABLE_DENOISER = True                  # OptiX denoiser cleans up noise from low SPP
RT_SUBFRAMES = 1                        # subframes per capture (in PathTracing, 1 = render SPP samples once)
STAGE_TIME_CODES_PER_SECOND = 120       # Timeline rate = capture rate (one frame per update)
PHYSICS_INTERNAL_RATE = 480             # PhysX internal solver rate (Hz). 480/120 = 4 substeps per capture frame.
PHYSX_POSITION_ITERATIONS = 4           # Solver position-correction passes per internal substep
PHYSX_VELOCITY_ITERATIONS = 4           # Solver velocity-correction passes per internal substep

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


def setup_stage():

    # Setup stage
    stage = stage_utils.create_new_stage(template="gridroom")
    configure_renderer(stage)

    # Define basic physic
    # Create the physics scene and configure PhysX solver iterations for better stability
    rep.functional.physics.create_physics_scene(
        "/PhysicsScene", timeStepsPerSecond=PHYSICS_INTERNAL_RATE
    )
    physx_scene_api = PhysxSchema.PhysxSceneAPI.Apply(
        stage.GetPrimAtPath("/PhysicsScene")
    )
    physx_scene_api.CreateMaxPositionIterationCountAttr(PHYSX_POSITION_ITERATIONS)
    physx_scene_api.CreateMaxVelocityIterationCountAttr(PHYSX_VELOCITY_ITERATIONS)

    return stage
