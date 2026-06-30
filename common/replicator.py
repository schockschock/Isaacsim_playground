"""Replicator camera, writer, and capture helpers."""
import os

import carb
import carb.settings
import omni.replicator.core as rep

from common.geometry import compute_eye_positions

OUTPUT_BASE = os.path.join(os.path.dirname(os.path.dirname(__file__)), "_output")
CAM_CLIPPING_RANGE = (0.01, 10000.0)

RENDER_MODE = "PathTracing"
PATH_TRACING_SPP = 32
ENABLE_DENOISER = True

STEREO_BASELINE = 0.06


def setup_camera(name="Camera",
                 parent="/World",
                 focal_length=24.0,
                 position=(1.0, 1.0, 1.0),
                 look_at=(0, 0, 0.5)):
    rep.functional.create.scope(name="Cameras", parent=parent)
    cam = rep.functional.create.camera(
        parent=f"{parent}/Cameras",
        name=name,
        clipping_range=CAM_CLIPPING_RANGE,
        focal_length=focal_length,
        position=position,
        look_at=look_at,
    )
    return cam


def setup_writer(output_subdir, rgb=True, semantic_segmentation=False,
                 instance_segmentation=False, distance_to_image_plane=False,
                 normals=False):
    out_dir = os.path.join(OUTPUT_BASE, output_subdir)
    os.makedirs(out_dir, exist_ok=True)
    backend = rep.backends.get("DiskBackend")
    backend.initialize(output_dir=out_dir)
    writer = rep.writers.get("BasicWriter")
    writer.initialize(
        backend=backend,
        rgb=rgb,
        semantic_segmentation=semantic_segmentation,
        instance_segmentation=instance_segmentation,
        distance_to_image_plane=distance_to_image_plane,
        normals=normals,
    )
    return writer

def setup_replicator(cam_resolution=(720, 720), output_subdir="replicator_output"):
    rep.orchestrator.set_capture_on_play(False)
    print("Setting up replicator...")
    cam = setup_camera()
    rp = rep.create.render_product(cam, cam_resolution, name="MainRenderProduct")
    writer = setup_writer(output_subdir)
    writer.attach(rp)
    return cam, writer, rp


def setup_stereo_rig(base_position, look_at, resolution,
                     baseline=STEREO_BASELINE, focal_length=24.0,
                     output_subdir="replicator_output"):
    """Create a toed-in two-camera stereo rig and attach a BasicWriter.

    Both cameras share the same look_at target (toed-in convergence). Eye
    positions are computed by common.geometry.compute_eye_positions (pure,
    unit-tested). Renderer settings MUST be applied before calling this,
    because rep.create.render_product snapshots the current renderer config.

    BasicWriter.attach([left_rp, right_rp]) writes one subdir per eye
    (named after the render product) under _output/<output_subdir>/.

    Args:
        base_position: rig center (x, y, z).
        look_at: shared convergence target (x, y, z).
        resolution: (width, height) for both render products.
        baseline: eye separation in meters (default STEREO_BASELINE).
        focal_length: camera focal length in mm.
        output_subdir: subdirectory under _output/ for this scenario.

    Returns:
        (left_cam, right_cam, [left_rp, right_rp], writer).
    """
    rep.orchestrator.set_capture_on_play(False)
    left_pos, right_pos = compute_eye_positions(base_position, look_at, baseline)

    rep.functional.create.scope(name="Cameras", parent="/World")
    left_cam = rep.functional.create.camera(
        parent="/World/Cameras", name="Left",
        clipping_range=CAM_CLIPPING_RANGE, focal_length=focal_length,
        position=left_pos, look_at=look_at,
    )
    right_cam = rep.functional.create.camera(
        parent="/World/Cameras", name="Right",
        clipping_range=CAM_CLIPPING_RANGE, focal_length=focal_length,
        position=right_pos, look_at=look_at,
    )

    left_rp = rep.create.render_product(left_cam, resolution, name="Left")
    right_rp = rep.create.render_product(right_cam, resolution, name="Right")

    writer = setup_writer(output_subdir)
    writer.attach([left_rp, right_rp])
    return left_cam, right_cam, [left_rp, right_rp], writer