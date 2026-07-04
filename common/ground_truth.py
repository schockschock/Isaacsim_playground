"""Ground-truth post-processing: disparity from stereo depth.

Pure-logic (numpy only, no Isaac deps). Computes disparity from a pair of
depth maps produced by a parallel-shift rectified stereo rig (ADR-0004).

With parallel optical axes and identical intrinsics:
    d(u, v) = f_px * B / Z(u, v)

where:
    f_px = focal_length_mm * resolution_x / aperture_x_mm  (pixel focal length)
    B    = stereo baseline in metres
    Z    = depth (distance along optical axis) in metres, from
           ``distance_to_image_plane`` annotator
"""
import os
import glob
import json
import numpy as np


def derive_f_px(focal_length_mm, resolution, aperture):
    return float(focal_length_mm) * float(resolution[0]) / float(aperture[0])


def compute_disparity(left_depth, right_depth, baseline_m, f_px):
    Z = np.asarray(left_depth, dtype=np.float32)
    with np.errstate(divide="ignore", invalid="ignore"):
        d = np.float32(f_px * baseline_m) / Z
        d[np.isinf(d) | (Z <= 0)] = 0.0
    return d


def post_process_disparity(output_root, baseline_m):
    """Read camera params and L+R depth maps from disk, compute+save disparity.

    Expects BasicWriter output layout:
        <output_root>/Left/camera_params/camera_params_0001.json
        <output_root>/Left/distance_to_image_plane/distance_to_image_plane_%04d.npy
        <output_root>/Right/distance_to_image_plane/distance_to_image_plane_%04d.npy

    Saves disparity maps under:
        <output_root>/disparity/disparity_%04d.npy
    """
    left_depth_dir = os.path.join(output_root, "Left", "distance_to_image_plane")
    right_depth_dir = os.path.join(output_root, "Right", "distance_to_image_plane")

    if not os.path.isdir(left_depth_dir):
        print(f"  [SKIP] disparity: no depth at {left_depth_dir}")
        return

    left_files = sorted(glob.glob(os.path.join(left_depth_dir, "*.npy")))
    if not left_files:
        print("  [SKIP] disparity: empty depth directory")
        return

    # Derive f_px from the first camera_params JSON
    cam_params_dir = os.path.join(output_root, "Left", "camera_params")
    json_files = sorted(glob.glob(os.path.join(cam_params_dir, "*.json")))
    if not json_files:
        print("  [SKIP] disparity: no camera_params found")
        return

    with open(json_files[0]) as f:
        params = json.load(f)
    focal_mm = params["cameraFocalLength"]
    aperture = params["cameraAperture"]
    resolution = params["renderProductResolution"]
    f_px = derive_f_px(focal_mm, resolution, aperture)

    disparity_dir = os.path.join(output_root, "disparity")
    os.makedirs(disparity_dir, exist_ok=True)

    for i, lf in enumerate(left_files):
        left_depth = np.load(lf)
        rf = lf.replace("Left", "Right") if "Left" in lf else os.path.join(
            right_depth_dir, os.path.basename(lf)
        )
        right_depth = np.load(rf) if os.path.isfile(rf) else left_depth
        disparity = compute_disparity(left_depth, right_depth, baseline_m, f_px)
        out_name = f"disparity_{i:04d}.npy"
        np.save(os.path.join(disparity_dir, out_name), disparity)

    print(f"  wrote {len(left_files)} disparity maps to {disparity_dir}")
