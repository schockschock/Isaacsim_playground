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

STEREO_BASELINE = 0.08
_PARALLEL_LOOK_AHEAD = 1000.0


def _sub(a, b):
    return tuple(x - y for x, y in zip(a, b))


def _add(a, b):
    return tuple(x + y for x, y in zip(a, b))


def _scale(v, s):
    return tuple(c * s for c in v)


def _length(v):
    return (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]) ** 0.5


def _normalize(v):
    n = _length(v)
    if n == 0.0:
        return (0.0, 0.0, 0.0)
    return tuple(c / n for c in v)


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


def setup_writer(output_subdir, output_base=None, rgb=True,
                 semantic_segmentation=False, instance_segmentation=False,
                 distance_to_image_plane=False, normals=False,
                 motion_vectors=False, camera_params=False,
                 bounding_box_2d_tight=False, bounding_box_3d=False,
                 colorize_semantic_segmentation=False,
                 colorize_instance_segmentation=False):
    base = output_base if output_base is not None else OUTPUT_BASE
    out_dir = os.path.join(base, output_subdir)
    os.makedirs(out_dir, exist_ok=True)
    backend = rep.backends.get("DiskBackend")
    backend.initialize(output_dir=out_dir)
    writer = rep.writers.get("BasicWriter")
    writer.initialize(
        backend=backend,
        rgb=rgb,
        semantic_segmentation=semantic_segmentation,
        colorize_semantic_segmentation=colorize_semantic_segmentation,
        instance_segmentation=instance_segmentation,
        colorize_instance_segmentation=colorize_instance_segmentation,
        distance_to_image_plane=distance_to_image_plane,
        normals=normals,
        motion_vectors=motion_vectors,
        camera_params=camera_params,
        bounding_box_2d_tight=bounding_box_2d_tight,
        bounding_box_3d=bounding_box_3d,
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
                     output_subdir="replicator_output", output_base=None,
                     enable_semantic_segmentation=True):
    """Create a parallel-shift two-camera stereo rig and attach a BasicWriter.

    Both cameras share the same forward direction (parallel optical axes, no
    toed-in convergence). Eye positions are shifted by +/- baseline/2 along
    the rig's right vector, and each camera looks along the forward axis at
    a virtual point _PARALLEL_LOOK_AHEAD metres ahead.

    Renderer settings MUST be applied before calling this, because
    rep.create.render_product snapshots the current renderer config.

    BasicWriter.attach([left_rp, right_rp]) writes one subdir per eye
    (named after the render product) under <output_base>/<output_subdir>/.

    Args:
        base_position: rig center (x, y, z).
        look_at: target to define the forward axis (forward = look_at - base).
        resolution: (width, height) for both render products.
        baseline: eye separation in meters (default 0.08 m).
        focal_length: camera focal length in mm.
        output_subdir: subdirectory under output_base for this scenario.
        output_base: root output directory (defaults to repo _output/).
        enable_semantic_segmentation: enable semantic segmentation annotator.

    Returns:
        (left_cam, right_cam, [left_rp, right_rp], writer).
    """
    rep.orchestrator.set_capture_on_play(False)

    forward = _normalize(_sub(look_at, base_position))
    left_pos, right_pos = compute_eye_positions(base_position, look_at, baseline)

    # Parallel-shift: each camera looks along the same forward direction
    left_look_at = _add(left_pos, _scale(forward, _PARALLEL_LOOK_AHEAD))
    right_look_at = _add(right_pos, _scale(forward, _PARALLEL_LOOK_AHEAD))

    rep.functional.create.scope(name="Cameras", parent="/World")
    left_cam = rep.functional.create.camera(
        parent="/World/Cameras", name="Left",
        clipping_range=CAM_CLIPPING_RANGE, focal_length=focal_length,
        position=left_pos, look_at=left_look_at,
    )
    right_cam = rep.functional.create.camera(
        parent="/World/Cameras", name="Right",
        clipping_range=CAM_CLIPPING_RANGE, focal_length=focal_length,
        position=right_pos, look_at=right_look_at,
    )

    left_rp = rep.create.render_product(left_cam, resolution, name="Left")
    right_rp = rep.create.render_product(right_cam, resolution, name="Right")

    writer = setup_writer(
        output_subdir,
        output_base=output_base,
        semantic_segmentation=enable_semantic_segmentation,
        instance_segmentation=True,
        distance_to_image_plane=True,
        normals=True,
        motion_vectors=True,
        camera_params=True,
        bounding_box_2d_tight=True,
        bounding_box_3d=True,
        colorize_semantic_segmentation=False,
        colorize_instance_segmentation=False,
    )
    writer.attach([left_rp, right_rp])
    return left_cam, right_cam, [left_rp, right_rp], writer
