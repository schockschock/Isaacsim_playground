"""Replicator camera, writer, and capture helpers."""
import os

import carb
import carb.settings
import omni.replicator.core as rep

OUTPUT_BASE = os.path.join(os.path.dirname(os.path.dirname(__file__)), "_output")
CAM_CLIPPING_RANGE = (0.01, 10000.0)

RENDER_MODE = "PathTracing"
PATH_TRACING_SPP = 32
ENABLE_DENOISER = True


def configure_renderer():
    """Apply RTX renderer settings before creating any render product.

    Render settings MUST be set before rep.create.render_product() is called,
    because the render product captures the current renderer configuration at
    creation time.
    """
    settings = carb.settings.get_settings()
    settings.set("/rtx/rendermode", RENDER_MODE)
    settings.set("/rtx/pathtracing/spp", PATH_TRACING_SPP)
    settings.set("/rtx/pathtracing/totalSpp", PATH_TRACING_SPP)
    settings.set("/rtx/pathtracing/optixDenoiser/enabled", ENABLE_DENOISER)

def setup_camera(name="Camera", parent="/World", focal_length=24.0):
    #configure_renderer()
    rep.functional.create.scope(name="Cameras", parent=parent)
    cam = rep.functional.create.camera(
        parent=f"{parent}/Cameras",
        name=name,
        clipping_range=CAM_CLIPPING_RANGE,
        focal_length=focal_length,
        position=(1.0, 1.0, 1.0),
        look_at=(0, 0, 0.5),
    )
    return cam


def setup_writer(output_subdir):
    out_dir = os.path.join(OUTPUT_BASE, output_subdir)
    os.makedirs(out_dir, exist_ok=True)
    backend = rep.backends.get("DiskBackend")
    backend.initialize(output_dir=out_dir)
    writer = rep.writers.get("BasicWriter")
    writer.initialize(
        backend=backend,
        rgb=True,
    )
    return writer, out_dir
