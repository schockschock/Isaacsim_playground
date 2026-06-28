"""Stage setup: ground plane, dome light (shadows off), physics scene."""
import omni.replicator.core as rep
import omni.usd
from pxr import Gf, PhysxSchema, Sdf, UsdGeom, UsdPhysics, UsdLux
import isaacsim.core.experimental.utils.stage as stage_utils


DOME_LIGHT_INTENSITY = 1000.0
DOME_LIGHT_COLOR = (1.0, 1.0, 1.0)
DOME_SHADOWS_ENABLED = False
DISTANT_LIGHT_INTENSITY = 180.0
ROTATION = (0, 60, 0)

def setup():
    stage = stage_utils.create_new_stage(template="gridroom")
    rep.functional.physics.create_physics_scene("/PhysicsScene", timeStepsPerSecond=60)

    return stage
